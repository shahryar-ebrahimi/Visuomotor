
import time
import robot.interface as robot
import os
import subprocess

cwd = os.getcwd()

duration = 60 # Duration of capturing data
name = './data/Inertia_calib_200Hz.dat'


robot.load()


print("Press <ENTER> to start capturing data")
input()

print("Start to move all around")
subprocess.Popen(['aplay', cwd + '/start.wav'])

robot.start_log(name, 19)

time.sleep(duration)
robot.stop_log()

subprocess.Popen(['aplay', cwd + '/start.wav'])
time.sleep(2)
robot.stay()
robot.unload()
