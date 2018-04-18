from perched import *
from worldmap import *
from worldmap_viewer import WorldMapViewer

from time import sleep

num_cams = 2
origin_id = 2
perched_cameras = [1,2]

perched_thread = PerchedCameraThread(verbose = False)
perched_thread.start_perched_camera_thread(perched_cameras)

world_map = WorldMap(origin_id = 2)
worldmap_viewer = WorldMapViewer(world_map=world_map);
worldmap_viewer.start();

client_threads = []

a = True
while True:
    world_map.update_perched_cameras(perched_thread.local_cameras)
    sleep(5)
    if a:
        worldmap_viewer.start()
    print(world_map.objects)
