from cozmo_server import *
import math
origin_id = 41
# perched_cameras = [1]

perched_thread = PerchedCameraThread(verbose = True)
# perched_thread.start_perched_camera_thread(perched_cameras)
perched_thread.start_perched_camera_thread()

world_map = WorldMap(origin_id = origin_id)
world_map.objects[origin_id] = ArucoMarkerObj([], origin_id, alpha=math.pi/2)
worldmap_viewer = WorldMapViewer(world_map=world_map);
worldmap_viewer.start();
server = Server(world_map, perched_thread=perched_thread)
client_threads = []

a = True
worldmap_viewer.start()
while True:
    world_map.update_perched_cameras(perched_thread.local_cameras)
    
    print("Print worldmap objects", world_map.objects)
    # for items in world_map.objects.keys()
    for key, value in world_map.objects.items():
        print("Type of key ", type(key))
        print("Key is ", key)
        if "LightCubeForeignObj-1" in str(key):
            cube1 = world_map.objects[key]
            print("Cube1 is ", cube1)
            print("Type of object is ", type(cube1))
        elif "LightCubeForeignObj-2" in str(key):
            cube2 = world_map.objects[key]
            print("Cube2 is ", cube2)
            print("Type of object is ", type(cube2))
        elif "LightCubeForeignObj-3" in str(key):
            cube3 = world_map.objects[key]
            print("Cube3 is ", cube3)
            print("Type of object is ", type(cube3))
            # print("Updating the Foreign Cube1 finally!")
            # world_map.update_cube(world_map,cube)
            # print("Updated cube successfully!")
    print("Out of FOR LOOP!")


    try:
        if cube1 is not None:
            print("Updating the Foreign Cube1 finally!")
            world_map.update_cube(world_map,cube1)
            break
        if cube2 is not None:
            print("Updating the Foreign Cube2 finally!")
            world_map.update_cube(world_map,cube2)
            break
        if cube3 is not None:
            print("Updating the Foreign Cube3 finally!")
            world_map.update_cube(world_map,cube3)
            break
    except Exception as e:
        print("Could not update the cubes because of ", str(e))


    print("Print shared worldmap objects", world_map.shared_objects)
    print("Print Camera Landmark pool", server.camera_landmark_pool)
    sleep(5)
