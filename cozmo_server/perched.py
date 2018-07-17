import cv2
from cv2 import aruco
import threading
import collections
from numpy import matrix, array, ndarray, sqrt, arctan2, pi
from time import sleep

from .transform import wrap_angle, rotationMatrixToEulerAngles

# Microsoft HD ( Calibrated to death )
microsoft_HD_webcam_cameraMatrix = matrix([[2943.432,       0.000000,    365.462],
                               [0.000000,   2954.628,    381.123],
                               [0.000000, 0.000000, 1.000000]])

microsoft_HD_webcam_distCoeffs = array([1.0877, -95.22934, 0.036528, -0.0568529, -3.144701])

class Cam():
    def __init__(self,cap,x,y,z,phi,theta):
        self.cap = cap
        self.x = x
        self.y = y
        self.z = z
        self.phi = phi
        self.theta = theta

    def __repr__(self):
        return '<Cam (%.2f, %.2f, %.2f)> @ %.2f' % \
               (self.x, self.y, self.z,self.phi*180/pi)

class PerchedCameraThread(threading.Thread):
    def __init__(self, verbose = True, viewer=True):
        threading.Thread.__init__(self)
        self.use_perched_cameras = False
        self.perched_cameras = []
        self.verbose = verbose
        self.viewer = viewer

        # Set camera parameters. (Current code assumes same parameters for all cameras connected to a computer.)
        self.cameraMatrix = microsoft_HD_webcam_cameraMatrix
        self.distCoeffs = microsoft_HD_webcam_distCoeffs
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters =  aruco.DetectorParameters_create()

        # camera landmarks from local cameras
        self.local_cameras = {}

        # camera landamrks from network (sent from server)
        self.global_cameras = {}

        self.rotation_matrices = {}

    # run() is invoked after thread.start()
    def run(self):
        while(True):
            if self.use_perched_cameras:
                print("Using perched cameras")
                self.process_image()
                # Computer overloaded if not given break
                sleep(0.05)
            else:
                print("not Using perched cameras")
                break

    def start_perched_camera_thread(self):
        # if not isinstance(cameras,list):
        #     cameras = [cameras]
        self.use_perched_cameras=True
        # self.perched_cameras = []
        # for x in cameras:
        #     cap = cv2.VideoCapture(x)
        #     if cap.isOpened():
        #         self.perched_cameras.append(cap)
        #     else:
        #        raise RuntimeError("Could not open camera %s." % repr(x))
        # for cap in self.perched_cameras:
        #     # hack to set highest resolution
        #     cap.set(3,4000) # Setting frame width to 4000
        #     cap.set(4,4000) # Setting frame height to 4000
        print("Particle filter now using perched cameras")
        self.start()

    def stop_perched_camera_thread(self):
        self.use_perched_cameras=False
        sleep(1)
        # for cap in self.perched_cameras:
        #     cap.release()
        print("Particle filter stopped using perched cameras")

    ''' function process_image: updates the camera landmarks from local cameras and saves it in self.local_cameras
    '''
    def process_image(self):
        # Dict with key: aruco id with values as cameras that can see the marker
        self.temp_cams = {}     # Necessary, else self.cameras is empty most of the time
        count = 0
        # for cap in self.perched_cameras:
            # Clearing Buffer by grabbing five frames
            # for i in range(5):
            #     cap.grab()
        frame = cv2.imread('scene.jpg')
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # cv2.imwrite('frame.jpg', gray)
        # if frame == None:
        #     raise Exception("could not load image !")
        # gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        # cv2.imwrite("trial.png", gray)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        print("Corners are"+str(corners))
        print("Ids are"+str(ids))
        # print("rejectedImgPoints are"+str(rejectedImgPoints))

        if type(ids) is ndarray:
            print("ids are ndarray")
            vecs = aruco.estimatePoseSingleMarkers(corners, 50, self.cameraMatrix, self.distCoeffs)
            rvecs, tvecs = vecs[0], vecs[1]
            print("Shape of rvecs is: ", rvecs.shape)
            print("Rvecs[0] is: ", rvecs[0])
            print("Rvecs[1] is: ", rvecs[1])
            print("Shape of tvecs is: ", tvecs.shape)
            print("Tvecs[0] is: ", tvecs[0])
            print("Tvecs[1] is: ", tvecs[1])
            for i in range(len(ids)):
                print("Range of for loop is ", len(ids))
                print("Id found is ",str(ids[i]))
                print("Shape of ids is ", ids.shape)
                print("Type ")

                # A 3X3 Rotation matrix is derived using cv2.Rodrigues() by giving as input the (1,3) dimensional rotation vector
                rotationm, jcob = cv2.Rodrigues(rvecs[i])

                # rvecs and tvecs are numpy arrays of shape (2,1,3) where 2 is the number of aruco markers identified
                # and rvecs[i] is a rotation vector of shape (1,3) corresponding to aruco marker correspoding at index i

                # ids is an nd array with shape (2,1) where 2 is the number of aruco markers identified

                    # transform to coordinate frame of aruco marker with id ids[i]
                transformed = matrix(rotationm).T*(-matrix(tvecs[i]).T)
                print("Shape of transformed vector is: ", transformed.shape)
                phi = rotationMatrixToEulerAngles(rotationm.T)

                    # add to temp_cams
                if ids[i][0] in self.temp_cams:
                    self.temp_cams[ids[i][0]]["Video 001"]=Cam("Video 001",transformed[0][0,0],
                        transformed[1][0,0],transformed[2][0,0],wrap_angle(phi[2]-pi/2), wrap_angle(phi[0]+pi/2))
                else:
                    self.temp_cams[ids[i][0]]={"Video 001":Cam("Video 001",transformed[0][0,0],
                        transformed[1][0,0],transformed[2][0,0],wrap_angle(phi[2]-pi/2), wrap_angle(phi[0]+pi/2))}
                if ids[i][0] in self.rotation_matrices:
                    self.rotation_matrices[ids[i][0]]["Video 001"] = matrix(rotationm).T
                else:
                    self.rotation_matrices[ids[i][0]]={"Video 001": matrix(rotationm).T}

        def deep_update(d, u):
            for k, v in u.items():
                if isinstance(v, collections.Mapping):
                    d[k] = deep_update(d.get(k, {}), v)
                else:
                    d[k] = v
            return d

        deep_update(self.local_cameras,self.temp_cams)
        if(self.verbose): print(self.local_cameras)
