from cozmo_server import *

origin_id = 2
perched_cameras = [1,2]

perched_thread = PerchedCameraThread(verbose = False)
perched_thread.start_perched_camera_thread(perched_cameras)

world_map = WorldMap(origin_id = 2)
worldmap_viewer = WorldMapViewer(world_map=world_map);
worldmap_viewer.start();

server = Server(world_map, perched_thread=perched_thread)
client_threads = []

a = True
while True:
    world_map.update_perched_cameras(perched_thread.local_cameras)
    sleep(5)
    if a:
        worldmap_viewer.start()
