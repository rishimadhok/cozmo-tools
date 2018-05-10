# Cozmo Server Module
The cozmo server module allows multiple cozmo robots, running simple_cli, to share a single world map. Cozmo Server has been designed to work with as many as 50 robots at once but has only been tested with two.
## Usage
### Server
On any computer on your local network navigate to the `cozmo_fsm` directory. You can run ```python3 server.py``` to run thedefault server. The default server will expect at least 3 cameras to be connected to this computer (it will assume the first one is a webcam facing the user and not attempt to use it) and the origin to be marked by an aruco marker with id `2`. You can modify the first few lines of the code to change which aruco marker and cameras to use. By default the server listens on port 1800

The line ```perched_cameras = [1, 2]``` can be changed to ```perched_cameras = [0, 1]``` to use the first 2 cameras or to ```perched_cameras = [1]``` to just use a single camera.  Other more advanced customizations can be made by adding parameters to the constructors; additional details can be found in the comments.

### Client
For the client to work there needs to be a running server on the same network as the client computer. (CMU_SECURE and the REL network are considered local) To start the client, in simple_cli run 
```start client <server_ip_address>```. When prompted enter the id of the aruco marker on your cozmo. This will start your connection. You can run ```start sharedmap``` to start using the map shared by the server. Clients can also run their own perched cameras and send perched camera data to the server by running ```start perched``` in simple_cli.

## How It Works
### Perched Cameras
The cozmo server relies on perched cameras to position robots with respect to the origin and eachother. The PerchedCameraThread runs as a parallel process to calculate the transformation between each camera and each aruco marker object. These can then be used together to find the transformation between aruco markers.
### Listener and Client Handler
The listener runs as a separate thread and waits for new connections. Once a robot connects to the server, it gets a dedicated ClientHandler that sends server data to the robot every 10 milliseconds and receives data about the robots surroundings and pose.
### Fusion
The fusion thread is where all the magic happens. This thread processess all camera information and computes the transformations between each aruco marker and the origin, as seen by each perched camera. It uses these transformations to come up with the most accurate current pose of the robot and fuses this with the pose information provided by the clients. Unfortunately, since the existing ParticleFilter code relies heavily on the robot object, we cannot use the particle filter to refine pose estimates as time goes by. This results to the robots jumping around every now and then as the system is unable to decide which camera data is more accurate. 


The fusion process also gets the landmarks from individual robots (lightcubes and wall objects) and places them on the global map. 

## Known Issues
* Perched cameras have too many false positive aruco markers. These are suppressed in ```client_handler.py```
* When a robot sees a wall it can get stuck in its current rotation. (It won't turn on the worldmap regardless of its pose in the real world)
* The transformation between the origin and robots is unreliable and robots can block the view of cameras. This can potentially be solved or alleviated by elevating the origin marker.
* On the client, pickle causes an import issue when cozmo_server is not in your python_path. This is solved by adding cozmo_server to path or running simple_cli from within the cozmo_fsm folder.
