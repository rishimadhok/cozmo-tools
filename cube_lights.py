import time

import cozmo
from cozmo.objects import LightCube1Id, LightCube2Id, LightCube3Id


def cozmo_program(robot: cozmo.robot.Robot):
    cube1 = robot.world.get_light_cube(LightCube1Id)  # looks like a paperclip
    cube2 = robot.world.get_light_cube(LightCube2Id)  # looks like a lamp / heart
    cube3 = robot.world.get_light_cube(LightCube3Id)  # looks like the letters 'ab' over 'T'

    if cube1 is not None:
        # cube1.set_lights(cozmo.lights.red_light)
        cube1.set_lights(cozmo.lights.Light(on_color=cozmo.lights.Color(rgb=(0,0,255)), off_color=cozmo.lights.Color(rgb=(0,0,0)), on_period_ms=250, off_period_ms=250, transition_on_period_ms=0, transition_off_period_ms=0))
    else:
        cozmo.logger.warning("Cozmo is not connected to a LightCube1Id cube - check the battery.")

    if cube2 is not None:
        cube2.set_lights(cozmo.lights.Light(on_color=cozmo.lights.Color(rgb=(0,0,255)), off_color=cozmo.lights.Color(rgb=(0,0,0)), on_period_ms=250, off_period_ms=250, transition_on_period_ms=0, transition_off_period_ms=0))
    else:
        cozmo.logger.warning("Cozmo is not connected to a LightCube2Id cube - check the battery.")

    if cube3 is not None:
        cube3.set_lights(cozmo.lights.Light(on_color=cozmo.lights.Color(rgb=(0,0,255)), off_color=cozmo.lights.Color(rgb=(0,0,0)), on_period_ms=250, off_period_ms=250, transition_on_period_ms=0, transition_off_period_ms=0))
    else:
        cozmo.logger.warning("Cozmo is not connected to a LightCube3Id cube - check the battery.")

    # Keep the lights on for 10 seconds until the program exits
    time.sleep(10)


cozmo.run_program(cozmo_program)