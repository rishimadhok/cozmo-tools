from cozmo_server import *

'''
from worldmap import *
from client_handler import ListenerThread
from worldmap_viewer import WorldMapViewer
import sys
from time import sleep
from . import perched
from perched import *

if '.' not in sys.path:
    sys.path.append('.')
'''
num_cams = 2
origin_id = 2
perched_cameras = [1,2]

perched_thread = PerchedCameraThread(verbose = False)
perched_thread.start_perched_camera_thread(perched_cameras)

world_map = WorldMap(origin_id = 2)
worldmap_viewer = WorldMapViewer(world_map=world_map);
worldmap_viewer.start();

listener = ListenerThread(world_map, perched=perched_thread)
listener.start()
client_threads = []

a = True
while True:
    world_map.update_perched_cameras(perched_thread.local_cameras)
    print(world_map.objects)
    sleep(5)
    if a:
        worldmap_viewer.start()
