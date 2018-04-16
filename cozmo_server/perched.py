import cv2
from cv2 import aruco
import threading
from numpy import matrix, array, ndarray, sqrt, arctan2, pi
from time import sleep

from transform import wrap_angle, rotationMatrixToEulerAngles

# Microsoft HD ( Calibrated to death )
microsoft_HD_webcam_cameraMatrix = matrix([[1148.00,       -3,    641.0],
                               [0.000000,   1145.0,    371.0],
                               [0.000000, 0.000000, 1.000000]])
microsoft_HD_webcam_distCoeffs = array([0.211679, -0.179776, 0.041896, 0.040334, 0.000000])

class Cam():
    def __init__(self,cap,x,y,z,phi, theta):
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
    def __init__(self, verbose = True):
        threading.Thread.__init__(self)
        self.use_perched_cameras = False
        self.perched_cameras = []
        self.verbose = verbose

        # Set camera parameters. (Current code assumes same parameters for all cameras connected to a computer.)
        self.cameraMatrix = microsoft_HD_webcam_cameraMatrix
        self.distCoeffs = microsoft_HD_webcam_distCoeffs
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters =  aruco.DetectorParameters_create()

        # camera landmarks from local cameras
        self.local_cameras = {}

        # camera landamrks from network (sent from server)
        self.global_cameras = {}

    def run(self):
        while(True):
            if self.use_perched_cameras:
                self.process_image()
                # Computer overloaded if not given break
                sleep(0.05)
            else:
                break

    def start_perched_camera_thread(self,cameras=[]):
        if not isinstance(cameras,list):
            cameras = [cameras]

        self.use_perched_cameras=True
        self.perched_cameras = []
        for x in cameras:
            cap = cv2.VideoCapture(x)
            if cap.isOpened():
                self.perched_cameras.append(cap)
            else:
               raise RuntimeError("Could not open camera %s." % repr(x))
        for cap in self.perched_cameras:
            # hack to set highest resolution
            cap.set(3,4000)
            cap.set(4,4000)
        print("Particle filter now using perched cameras")
        self.start()

    def stop_perched_camera_thread(self):
        self.use_perched_cameras=False
        sleep(1)
        for cap in self.perched_cameras:
            cap.release()
        print("Particle filter stopped using perched cameras")

    ''' function process_image: updates the camera landmarks from local cameras and saves it in self.local_cameras
    '''
    def process_image(self):
        # Dict with key: aruco id with values as cameras that can see the marker
        self.temp_cams = {}     # Necessary, else self.cameras is empty most of the time
        for cap in self.perched_cameras:
            # Clearing Buffer by grabbing five frames
            for i in range(5):
                cap.grab()
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if type(ids) is ndarray:
                vecs = aruco.estimatePoseSingleMarkers(corners, 50, self.cameraMatrix, self.distCoeffs)
                rvecs, tvecs = vecs[0], vecs[1]
                for i in range(len(ids)):
                    rotationm, jcob = cv2.Rodrigues(rvecs[i])

                    # transform to coordinate frame of aruco marker with id ids[i]
                    transformed = matrix(rotationm).T*(-matrix(tvecs[i]).T)
                    phi = rotationMatrixToEulerAngles(rotationm.T)

                    #add to temp_cams
                    if ids[i][0] in self.temp_cams:
                        self.temp_cams[ids[i][0]][str(cap)]=Cam(str(cap),transformed[0][0,0],
                            transformed[1][0,0],transformed[2][0,0],wrap_angle(phi[2]-pi/2), wrap_angle(phi[0]+pi/2))
                    else:
                        self.temp_cams[ids[i][0]]={str(cap):Cam(str(cap),transformed[0][0,0],
                            transformed[1][0,0],transformed[2][0,0],wrap_angle(phi[2]-pi/2), wrap_angle(phi[0]+pi/2))}
        self.local_cameras = self.temp_cams
        if(self.verbose): print(self.local_cameras)
