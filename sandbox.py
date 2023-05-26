

import time
import robot.interface as robot



robot.load()

print("Loaded")

tx,ty=.15,.10
print("About to move to point")
input()
robot.move_to(tx,ty,3.)

while not robot.move_is_done():
    time.sleep(.1)

print("Done move")
input()

robot.unload()
