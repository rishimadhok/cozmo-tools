from threading import Thread
from time import sleep
from copy import deepcopy

import socket
import pickle
import numpy as np
from .worldmap import *
from .transform import rotationMatrixToEulerAngles

class fakeParticleFilter():
    def __init__(self):
        self.sigma_r = 50
        self.sigma_alpha = 15 * (pi/180)
        self.sigma_phi = 15 * (pi/180)
        self.sigma_theta =  15 * (pi/180)
        self.sigma_z = 50
        self.landmark_sensor_variance_Qt = np.array([[self.sigma_r**2, 0                  , 0],
                                                     [0              , self.sigma_alpha**2, 0],
                                                     [0              , 0                  , self.sigma_phi**2]])
        # variance of camera location (cylindrical coordinates)
        # phi is the angle around the Z axis of the robot
        # theta is the angle around the X axis of the camera (pitch)
        self.camera_sensor_variance_Qt = np.array([[self.sigma_r**2 , 0                  , 0               ,0                , 0],
                                              [0               , self.sigma_alpha**2, 0               ,0                , 0],
                                              [0               , 0                  , self.sigma_z**2 ,0                , 0],
                                              [0               , 0                  , 0               ,self.sigma_phi**2, 0],
                                              [0               , 0                  , 0               ,0                , self.sigma_theta**2]])

    def sensor_jacobian_H_cam(self, dx, dy, dist):
        """Jacobian of sensor values (r, alpha) wrt particle state x,y
           where (dx,dy) is vector from particle to lm, and
           r = sqrt(dx**2 + dy**2), alpha = atan2(dy,dx), z = z, phi = phi, theta = theta"""
        q = dist**2
        sqr_q = dist
        return np.array([[dx/sqr_q, dy/sqr_q, 0, 0, 0],
                         [-dy/q   , dx/q    , 0, 0, 0],
                         [0       , 0       , 1, 0, 0],
                         [0       , 0       , 0, 1, 0],
                         [0       , 0       , 0, 0, 1],])

    def make_landmark(self, H, cam):
        lm_mu = np.array([[H[0,3]],[H[1,3]]])
        eu = rotationMatrixToEulerAngles(H)
        pass



class Server():
    def __init__(self, worldmap, port=1800, perched_thread=None, aruco_offset=0):
        #Worldmap (must have origin_id set)
        self.world_map = worldmap

        #Rotation of the origin aruco marker (0 by default)
        self.aruco_offset_angle = aruco_offset


        self.camera_landmark_pool = {}
        self.poses = {}
        self.foreign_objects = {}

        self.perched_thread = perched_thread
        self.listener = ListenerThread(worldmap,port,perched_thread,self)
        self.listener.start()
        self.particle = fakeParticleFilter()
        self.fusion = FusionThread(self)
        self.fusion.start()
class FusionThread(Thread):
    def __init__(self, server):
        super().__init__()
        self.server = server
        self.origin_id = server.world_map.origin_id
        self.accurate = {}
        self.transforms = {}

    ''' function find_accurate:
        iterates through self.server.camera_landmark_pool to find which of the landmark locations
        perceived by the cameras are more accurate and updates self.accurate
        returns whether or not self.accurate is updated
    '''
    def find_accurate(self):
        flag = False
        for aruco_id1, camera_pool1 in self.server.camera_landmark_pool.items():
            for aruco_id2, camera_pool2 in self.server.camera_landmark_pool.items():
                if aruco_id1 == aruco_id2:
                    continue
                for camera, landmark in camera_pool1.items(): #how each camera sees aruco_id1
                    if camera in camera_pool2: #if the same camera sees aruco_id2
                        varsum = landmark[2].sum()+camera_pool2[camera][2].sum() #total error
                        if varsum < self.accurate.get((aruco_id1,aruco_id2),(inf,None))[0]: #if error improves with this camera
                            self.accurate[(aruco_id1,aruco_id2)] = (varsum,camera) #update camera, error/accuracy
                            flag = True
        return flag

    ''' function find_transforms:
        finds the transformations from origin to landmark location, updating
        self.transforms with the locations of landmarks.
    '''
    def find_transforms(self):
        for (aruco_id1, aruco_id2), (varsum, camera) in self.accurate.items():
            #landmark_pool[aruco_id][camera] is the aruco landmark with aruco_id as seen by camera
            x1,y1 = self.server.camera_landmark_pool[aruco_id1][camera][0]
            h1,p1,t1 = self.server.camera_landmark_pool[aruco_id1][camera][1]
            x2,y2 = self.server.camera_landmark_pool[aruco_id2][camera][0]
            h2,p2,t2 = self.server.camera_landmark_pool[aruco_id2][camera][1]

            #theta is the change in angle between aruco and us (aruco is at an angle of pi/2)
            theta_t = wrap_angle(p2 - p1 + self.server.aruco_offset_angle)
            x_t = x2 - (x1*cos(theta_t) + y1*sin(theta_t))
            y_t = y2 - (-x1*sin(theta_t) + y1*cos(theta_t))
            self.transforms[(aruco_id1,aruco_id2)] = (x_t, y_t, theta_t, camera)

    def run(self):
        while(True):
            #adding local camera landmarks into camera_landmark_pool
            try:
                #Iterate through all aruco_ids to find their set of rotation matrices
                for aruco_id, rm in self.server.perched_thread.rotation_matrices.items():
                    #for each camera that sees this aruco marker, get the rotation matrix
                    for camera, rotationm in rm.items():
                        #camera object w.r.t aruco_id
                        try:
                            cam = self.server.perched_thread.local_cameras[aruco_id][camera]
                            x = cam.x
                            y = cam.y
                            z = cam.z

                            #Get rotation matrix & distances from camera to origin aruco frame
                            cam_origin = self.server.perched_thread.local_cameras[self.origin_id][camera]
                            rm_origin = self.server.perched_thread.rotation_matrices[self.origin_id][camera]

                            #Transformation matrix from aruco frame to camera frame
                            H1 = np.matrix(
                                 [[rotationm[0,0], rotationm[0,1], rotationm[0,2], x],
                                  [rotationm[1,0], rotationm[1,1], rotationm[1,2], y],
                                  [rotationm[2,0], rotationm[2,1], rotationm[2,2], z],
                                  [0,0,0,1]])

                            #Transformation matrix from origin frame to camera frame
                            H2 = np.matrix(
                                 [[rm_origin[0,0], rm_origin[0,1], rm_origin[0,2], cam_origin.x],
                                  [rm_origin[1,0], rm_origin[1,1], rm_origin[1,2], cam_origin.y],
                                  [rm_origin[2,0], rm_origin[2,1], rm_origin[2,2], cam_origin.z],
                                  [0,0,0,1]])

                            #Transformation matrix from robot frame to origin frame
                            H = H2 @ np.linalg.inv(H1)

                            #Get euler angles from resulting rotation matrix
                            eu = rotationMatrixToEulerAngles(H[0:3,0:3])
                            
                            #pack everything into a landmark
                            lm_mu = np.array([[H[0,3]],[H[1,3]]])
                            lm_height = (0, eu[2], eu[0])
                            lm_sigma = self.server.particle.camera_sensor_variance_Qt

                            self.server.camera_landmark_pool[aruco_id] = {camera: (lm_mu, lm_height, lm_sigma)}

                        except Exception as e: pass
            except Exception as e: pass
            try:
                if self.find_accurate(): self.find_transforms()
            except Exception as e: print(repr(e))
            try:
                self.update_foreign_robot()
                self.update_foreign_objects()
            except Exception as e: print(repr(e))
            sleep(0.02)

    def update_foreign_objects(self):
        for key, value in self.transforms.items(): 
            try:
                if key[1] == self.origin_id:
                    x_t, y_t, theta_t, cap = value
                    for k, v in self.server.foreign_objects[key[0]].items():
                        x2 =  v.x*cos(theta_t) + v.y*sin(theta_t) + x_t
                        y2 = -v.x*sin(theta_t) + v.y*cos(theta_t) + y_t
                        if isinstance(k,str) and "Wall" in k:
                            # update wall
                            if k in self.server.world_map.objects:
                                if self.server.world_map.objects[k].is_foreign:
                                    self.server.world_map.objects[k].update(x=x2, y=y2, theta=wrap_angle(v.theta-theta_t))
                            else:
                                copy_obj = deepcopy(v)
                                copy_obj.x = x2
                                copy_obj.y = y2
                                copy_obj.theta = wrap_angle(v.theta-theta_t)
                                copy_obj.is_foreign = True
                                self.server.world_map.objects[k]=copy_obj
                        elif isinstance(k,str) and "Cube" in k:
                            # update cube
                            if k in self.server.world_map.objects:
                                if self.server.world_map.objects[k].is_foreign:
                                    self.server.world_map.objects[k].update(x=x2, y=y2, theta=wrap_angle(v.theta-theta_t))
                            else:
                                copy_obj = deepcopy(v)
                                copy_obj.x = x2
                                copy_obj.y = y2
                                copy_obj.theta = wrap_angle(v.theta-theta_t)
                                copy_obj.is_foreign = True
                                self.server.world_map.objects[k]=copy_obj
            except Exception as e: pass

    def update_foreign_robot(self):
        for key, value in self.transforms.items():
            try:
                if key[1] == self.origin_id:
                    x_t, y_t, theta_t, cap = value
                    x, y, theta = self.server.poses[key[0]]
                    x2 =  x*cos(theta_t) + y*sin(theta_t) + x_t
                    y2 = -x*sin(theta_t) + y*cos(theta_t) + y_t
                    # improve using update function instead of new obj everytime
                    self.server.world_map.objects["Foreign-"+str(key[0])]=RobotForeignObj(cozmo_id=key[0],
                                         x=x2, y=y2, z=0, theta=wrap_angle(theta-theta_t), camera_id = int(cap[-2]))
            except Exception as e: pass

''' class ListenerThread: listens on the specified port (1800 by default) and creates and runs new 
    ClientHandlerThreads for each new connection.
'''
class ListenerThread(Thread):
    def __init__(self, worldmap, port=1800, perched=None, server=None):
        super().__init__()
        self.port = port
        self.socket = None
        self.started = False
        self.foreign_objects = {}
        self.perched = perched
        self.world_map = worldmap
        self.server = server

    def run(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setblocking(True)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True) #enables server restart
        self.socket.bind(("", self.port))
        self.threads = []
        for i in range(50):
            self.socket.listen(5)
            c, addr = self.socket.accept()
            print("Got Connection From: ",addr)
            self.threads.append(ClientHandlerThread(i, c, self.perched, self.world_map, self.server))
            self.threads[i].start()

''' ClientHandlerThread: gets data from clients to store on server and sends the latest data back
'''
class ClientHandlerThread(Thread):
    def __init__(self, thread_id, client, perched, world_map, server):
        super().__init__()
        self.threadID = thread_id
        self.c = client
        self.perched = perched
        self.world_map = world_map
        self.c.sendall(pickle.dumps("Hello"))
        self.aruco_id = int(pickle.loads(self.c.recv(1024)))
        self.name = "Client-"+str(self.aruco_id)
        self.server = server
        #update camera landmark pool???
        self.server.camera_landmark_pool[self.aruco_id] = {}
        self.to_send = {}
        print("Initialized thread for: ",self.name)

    def run(self):
        while True:
            #send from server to clients
            for key, value in self.world_map.objects.items():
                if isinstance(key,LightCube):
                    self.to_send["LightCubeForeignObj-"+str(value.id)]= LightCubeForeignObj(id=value.id, x=value.x, y=value.y, z=value.z, theta=value.theta)
                elif isinstance(key,str):
                    # Send walls and cameras
                    self.to_send[key] = value         # Fix case when object removed from shared map
            # append 'end' to end to mark end
            self.c.sendall(pickle.dumps([self.perched.local_cameras,self.to_send])+b'end')
            sleep(0.1)
            # hack to recieve variable size data without crashing
            data = b''
            while True:
                data += self.c.recv(1024)
                if data[-3:]==b'end':
                    break
            cams, landmarks, foreign_objects, pose = pickle.loads(data[:-3])
            for key, value in cams.items():
                if key in self.robo.server.perched_thread.camera_pool:
                    self.world_map.perched.camera_pool[key].update(value)
                else:
                    self.world_map.perched.camera_pool[key]=value
            self.server.camera_landmark_pool[self.aruco_id].update(landmarks)
            self.server.poses[self.aruco_id] = pose
            self.server.foreign_objects[self.aruco_id] = foreign_objects
