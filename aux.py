
import pygame
import time
import sys

import numpy as np
import pickle


import scipy


# The font used for displaying anything of interest
FONTFILE = "fonts/Aller_Rg.ttf"
mainFontSize = 24





def init_interface(conf):

    # Pygame configuration constants
    if conf['fullscreen']:
        displayFlags = pygame.FULLSCREEN # use for full-screen
    else:
        displayFlags = pygame.NOFRAME # for non-fullscreen (windowed mode), but without frame


    pygame.init()

    # Load the fonts
    #mainfont = pygame.font.Font(FONTFILE, mainFontSize)
    mainfont = None

    import os
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % conf["screenpos"] # controls where the subject screen will appear
    
    # Initialise the display
    screen = pygame.display.set_mode(conf["screensize"],displayFlags)
    screen.convert()
    pygame.display.set_caption('Curry')
    pygame.mouse.set_visible(False)

    if conf['use_joystick']:
        joystick_count = pygame.joystick.get_count()
        if joystick_count==0:
            print("## ERROR: no joystick found!")
            sys.exit(0) # this is problematic!
        if joystick_count>1:
            print("## ERROR: multiple joystick.  Unsure which to use.")
            sys.exit(0)
        
        conf['joystick'] = pygame.joystick.Joystick(0)
        conf['joystick'].init()
        conf['joystick_history']=[]
    
    screen.fill(conf["bgcolor"])

    pygame.display.flip()

    return (screen,mainfont)



def ending():
    # To stop the program
    pygame.quit()





def isfloat(value):
    # Check if the given variable is a float
    # I know, it's ugly, but it works.
    try:
        float(value)
        return True
    except ValueError:
        return False





def fade(p,colora,colorb):
    ## Fade between two colours, as determined by a ratio p (from 0 to 1).
    ## IF p==0, use color a, if p==1, use color b
    lst = (1-p)*np.array(colora)+p*np.array(colorb)
    return tuple([ int(x) for x in lst ])



def writeln(contents):
    # A simplified version for writing a line of output to file
    cols = []
    for (f,c) in contents:
        #print(f,c)
        if f==None:
            cols.append('nan')
        else:
            cols.append(('%'+c)%f)
    # cols = [ 'NA' if f==None else  for (f,c) in contents ]
    return (" ".join(cols)+"\n")







def robot_to_screen(x,y,conf):
    """ Given robot coordinates, find the screen coordinates. """
    #global conf
    calib=conf["calib"]
    return (int(calib["interc.x"] + (x*calib["slope.x"])),
            int(calib["interc.y"] + (y*calib["slope.y"])))



def screen_to_robot(rx,ry,conf):
    """ Given coordinates on the screen (in pixels), convert to robot coordinates. """
    #global conf
    calib=conf["calib"]
    return ( (rx-calib["interc.x"])/calib["slope.x"],
             (ry-calib["interc.y"])/calib["slope.y"])









def rotate(pts,ang,cnt):
    """ Rotate a point (x,y) in pts by a certain angle (in radians),
    around a certain pivot point (cnt) """
    # See e.g. http://gis.stackexchange.com/questions/23587/how-do-i-rotate-the-polygon-about-an-anchor-point-using-python-script
    if np.isnan(ang): return pts
    pts = np.array(pts)
    cnt = np.array(cnt)
    #ang = -ang
    return scipy.dot(pts-cnt,
                     # Rotation matrix
                     scipy.array([[np.cos(ang),np.sin(ang)],
                                  [-np.sin(ang),np.cos(ang)]]))+cnt
    




def deg2rad(deg):
    return (deg/180)*np.pi
