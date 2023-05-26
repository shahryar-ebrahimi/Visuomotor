# run instruction: python run_arc.py


import pygame
import numpy
import sys
import os

import time
import numpy as np
import random
import datetime
import math

# import h5py

import scipy

import threading
from threading import Thread

from aux import *

DEBUG = len(sys.argv)>1 and sys.argv[1]=='dummy'
if DEBUG:
    import robot.dummy as robot # for debug/development
else:
    import robot.interface as robot # for real testing

import subprocess

PYTHON3 = (sys.version_info > (3, 0))
if PYTHON3: # this really is tested on Python 3 so please use that
    from tkinter import *
    from tkinter import messagebox as tkMessageBox
    from tkinter import filedialog
    import tkinter.simpledialog as tksd
    from tkinter import messagebox
    import tkinter.ttk as ttk

else: # python2
    print("## WARNING: don't use Python 2. This code is designed for Python 3.")
    from Tkinter import * # use for python2
    import tkMessageBox
    from Tkinter import filedialog
    import tkSimpleDialog as tksd

import json
import pandas as pd
from pandas.api.types import is_string_dtype



from mouse import *

import subprocess # shr
import time # shr

import os # shr


# The little control window
CONTROL_WIDTH,CONTROL_HEIGHT= 600,700 #1000,800 #450,400 # control window dimensions
CONTROL_X,CONTROL_Y = 500,50 # controls where on the screen the control window appears


EXPERIMENT = "washout"



numpy.random.seed() # make sure we're really random
random.seed()

# This is a global dict that holds all the configuration options.
# Using a single variable for them makes it easier to keep track.
global conf
conf = {}


conf['N_ROBOT_LOG'] = 13 # how many columns go in the robot log file


conf['n_center_capture']=100 # how many samples to collect when capturing the center
conf['center_capture_period']=.01 # how long to wait between capturing samples for establishing the center


conf['fullscreen']=False

# The background of the screen
conf["bgcolor"] = (0,0,0) #(0,0,0)

# The size of the window (for pygame)
conf["screensize"] = (1920,1080) # inmotion screen
conf["screenpos"] = (1600,0) # the offset (allows you to place the subject screen "on" the second monitor)



# Read the screen-to-robot calibration from file
conf['calibfile']='calib.json'
if not os.path.exists(conf['calibfile']):
    print("## ERROR: cannot find screen calibration. You may want to run screencalib.py and save the calibration as %s"%conf['calibfile'])
    sys.exit(-1)
_,conf["calib"] = json.load(open(conf['calibfile'],'r')) #, encoding='latin1') if PYTHON3 else pickle.load(open('calib.pickle27','rb'))




# The sizes of various objects
conf["cursor_radius"] = .0035 # in robot coordinates (m)

# this is the display size of the target, not the size of the target area used for determining whether subjects are long enough "within" the target.
conf["target_radius"] = .0075
conf["target_colour"]  = (50,50,50) #(100,100,100)

# These are the colours for the target, when the subject is too slow, too fast, or correct
conf["target_slow_colour"]    = (0,0,255)
conf["target_correct_colour"] = (0,255,0)
conf["target_fast_colour"]    = (255,0,0)


# shr

# These are the tones for the target and the initial position, when the subject is too slow, too fast, or correct

cwd = os.getcwd()

conf["target_slow_tone"]    = cwd + '/slow.wav' 
conf["target_correct_tone"] = cwd + '/correct.wav' 
conf["target_fast_tone"]    = cwd + '/fast.wav' 
conf["start_tone"]          = cwd + '/start.wav' 

# shr





# Cutoffs on the peak velocity for determining whether the movement was too slow
conf['too_slow_vmax']=.35
conf['too_fast_vmax']=.45

# the marker for the center
conf['center_marker_radius']=.0075
conf['center_marker_colour']= conf["target_colour"] #(50,50,50) #75,75,75) #(100,100,100)

# The cursor colour
conf["active_cursor_colour"]  = (25,90,25)  # (0,255,0)
conf["passive_cursor_colour"] = (255,0,0)


# The radius of the movement
conf['movement_radius']=.08 # meters
conf['movement_small_range'] = .1 # meters

conf['min_movement_radius']=.75*conf['movement_radius'] # meters; the minimum distance a subject must have travelled before we can call the end of the movement based on peak velocity

conf['target_overshoot_SD']=.01 # in meters, the SD of the overshoot/undershoot of the target in passive trials
conf['target_overshoot_ask_p']=.25 # percentage of passive trials where we will randomly ask whether the cursor overshot or not


conf['robot_center']= (0,-.05) # robot center position
conf['robot_center_x'],conf['robot_center_y']=conf["robot_center"]  # just because we often need these separately as well


# The extent of the arc being displayed on the screen when the
# subject selects the placement of the hand
# This is in angles, degrees, with 0 = straight ahead, and positive angles are counterclockwise
conf['arc_range']= (-45,45)

conf['arc__segments']     = 100 # how many segments to draw the arc (higher=more precision but takes more resources)
conf['arc_thickness']     = .002 # how thick to draw the 'selector' arc
conf['arc_colour']        = (50,50,50) # colour of the arc selector


conf['selector_radius']=.002 # the radius of the selector ball that is controlled by the joystick (in m)
conf['selector_colour']=(255,0,0)
conf['selector_length']=.01 # the half length of the selector line (in m)
conf['selector_width']=4 # the width of the selector (in pixels)


# How long to take for the return movement
conf['return_duration']=1.5 # seconds

# How long to take for the passive movements
conf['passive_duration']=1.0 # seconds

conf['stay_duration']=1 # how long to stay "out there" in between forward and backward movement


# The review panel that shows you the subject's movement
conf['review_trajectory_colour'] =(255,255,255)
conf['review_rotated_colour']    =(0,255,0)
conf['review_linewidth']         =3
conf['review_force_scale']       =.01 # scale from N to m, just for display purposes
conf['review_force_colour']      =(255,0,0)
conf['review_force_width']       =3
conf['review_passive_width']     =3 # width of the line of the passive point
conf['review_passive_colour']    =(150,150,150)


# How much curl to use
conf['force_field_curl'] = 15 # N/(m/s)

conf['channel_stiffness'] = -4000.0
conf['channel_damping']   = 30.0


# Range of the joystick values
# anything outside this range is snapped to the edges
conf['use_joystick']=False
conf['max_joystick']= 1
conf['min_joystick']= 0



conf['use_mouse']=False
#conf['mouse_device']='/dev/input/by-id/usb-Kensington_Kensington_USB_PS2_Orbit-mouse'
conf['mouse_device']='/dev/input/by-id/usb-Microsoft_Microsoft_5-Button_Mouse_with_IntelliEye_TM_-mouse'
conf['mouse_selector_tick']=-.0005 # how much to change the selector (range 0..1) for one mouse 'tick' (this determines the maximum precision)


conf['pause_duration']=.5 # how long to hold at start when returning from the previous trial.
conf['recog_pause_duration']=1.5 # how long to hold at start in between recognition passive movements


# The controller that can be used for the fade duration
conf['fade_controller'] = 5
conf['fade_duration']=.5 # how long to fade when holding at the starting point
conf['fade_cue_colour']=(255,0,0) # the colour of the cursor while holding still (fading forces)

#conf['move_cue_colour']=(0,255,0) # the colour of the cursor when ready to move
conf['move_controller'] = 6


# What is the cursor behaviour when rotation=NA?
#  if na.cursor.show = False, the cursor will be hidden when rotation=NA and the cursor is outside the starting zone;
#  if it is True, the cursor will be shown, but visually error clamped to zero
conf['na.cursor.show'] = False

# Show a "horizontal" bar moving along with the subject in the rotation=NA condition?
conf['na.bar.show'] = True


conf['bar.color']=(255,255,0)
conf['bar.width']=3 # px
conf['bar.minx']=-.2
conf['bar.maxx']= .2
conf['bar.as.arc']=True # whether to show the bar as an arc


# The range of angles to show when we show an arc as a visual display
conf['recognition_display_arc']= True # whether to show an arc (rather than a bar) during the recognition test
conf['cursor_arc_range']=(-60,60)
conf['cursor_arc_segments']=15
conf['cursor_arc_thickness']=.01 # in robot coordinates (m)
conf['cursor_arc_colour']=(255,255,0) ## DEPRECATED -- close-to-zero chance of actually having any effect on anything


conf['motorcopy_forward_colour']=(180,180,180)


# Note that these phase numbers are not necessarily incremental...
conf['phases']=['init',
                'return',
                'pause',
                'forward',
                'backward',
                'completed',
                'move',
                'endmove',
                'select',
                'fade',
                'stay',
                'hold',
                'ask'
]

conf['python']=[sys.executable,sys.version,sys.prefix,sys.exec_prefix]

# Data that is specific to this trial
trialdata = {}





def conv_ang(a):
    """ Angles in this experiment are generally given in degrees
    w.r.t. straight ahead, just because that's simpler.
    But then sin and cos like to have them in radians, relative
    to straight right. So here we do that translation."""
    return ((a+90)/180)*np.pi





def draw_arc_selector(surf):
    """ Draw the arc on the screen along which the subjects can
    choose the felt position of the hand."""

    surf.fill(conf['bgcolor'])

    mna,mxa = conf['arc_range']
    minang,maxang = conv_ang(mna),conv_ang(mxa) # convert into usable angles
    angs = np.linspace(minang,maxang,conf['arc_draw_segments'])

    # Now make an inner and outer arc
    cx,cy = conf['robot_center']
    points = []
    for sgn in [-.5,.5]:
        rad = conf['movement_radius']-sgn*conf['arc_thickness']
        p = [ (cx+rad*np.cos(a),cy+rad*np.sin(a)) for a in angs ]
        if sgn<0: p.reverse()
        points += p


    # And then plot a polygon!
    poly = [ robot_to_screen(ry,rz,conf) for (ry,rz) in points ]
    pygame.draw.polygon( surf,conf['arc_colour'],poly)


    if 'selector_angle' in trialdata:
        #print(conf['selector_position'])
        sx,sy=position_from_angle(conv_ang(trialdata['selector_angle']),conf['movement_radius']-conf['selector_length'])
        shortpos = robot_to_screen(sx,sy,conf)
        lx,ly=position_from_angle(conv_ang(trialdata['selector_angle']),conf['movement_radius']+conf['selector_length'])
        longpos  = robot_to_screen(lx,ly,conf)
        pygame.draw.line(surf,conf['selector_colour'],shortpos,longpos,conf['selector_width'])
        #pos =
        #draw_ball(conf['screen'],pos,conf['selector_radius'],conf['selector_colour'])




def draw_bar(surface,ypos):
    """
    Let's draw a bar that moves along with the subject on the screen.
    The "vertical" position of the bar is indicated by ypos.
    """
    fromc = robot_to_screen(conf['bar.minx'],ypos,conf)
    toc   = robot_to_screen(conf['bar.maxx'],ypos,conf)

    pygame.draw.line(surface,conf['bar.color'],fromc,toc,conf['bar.width'])






def draw_arc(surf,dist,color):
    """
    Let's draw a bar that moves along with the subject on the screen.
    The "vertical" position of the bar is indicated by ypos.
    """
    mna,mxa = conf['cursor_arc_range']
    minang,maxang = conv_ang(mna),conv_ang(mxa) # convert into usable angles
    angs = np.linspace(minang,maxang,conf['cursor_arc_segments'])

    # Now make an inner and outer arc
    cx,cy = conf['robot_center']
    points = []
    for sgn in [-.5,.5]:
        rad = dist-sgn*conf['arc_thickness']
        p = [ (cx+rad*np.cos(a),cy+rad*np.sin(a)) for a in angs ]
        if sgn<0: p.reverse()
        points += p

    # And then plot a polygon!
    poly = [ robot_to_screen(ry,rz,conf) for (ry,rz) in points ]
    pygame.draw.polygon( surf,color,poly)









def draw_ball(surface,pos,radius,colour):
    """
    Draw one of the two target "balls" (at the edges of the arc)

    Arguments
    sgn is whether this is the left or right direction.
    colour is the draw colour
    """

    # Draw the two target balls
    #for sgn in [-1,1]:
    #tx,ty = conf["ARC_BASE_X"]+sgn*conf["ARC_RADIUS"],conf["ARC_BASE_Y"]
    tx,ty=pos
    x1,y1 = robot_to_screen(tx-radius,ty-radius, conf)
    x2,y2 = robot_to_screen(tx+radius,ty+radius, conf)
    minx=min(x1,x2)
    maxx=max(x1,x2)
    miny=min(y1,y2)
    maxy=max(y1,y2)
    pygame.draw.ellipse(surface,colour,#conf["ARC_COLOUR"],
                        [minx,miny,maxx-minx,maxy-miny])





def pinpoint():
    """ Select a position on the screen using the joystick."""
    # Start the main loop
    trialdata['redraw']=True
    launch_mainloop()





def hold_fade():
    """ Hold and fade the robot handle """
    robot.wshm('traj_count',          0) # ensure that we start recording at the beginning of the trajectory buffer
    robot.wshm('fvv_workspace_enter', 0) # initialise: signal that the subject has not yet entered the workspace
    robot.wshm('fvv_robot_center_x', conf['robot_center_x'])
    robot.wshm('fvv_robot_center_y', conf['robot_center_y'])
    robot.wshm('plg_p1x',            conf['robot_center_x']) # the position for the initial hold
    robot.wshm('plg_p2x',            conf['robot_center_y'])
    robot.wshm("fvv_force_fade",      1.0) # This starts at 1.0 but exponentially decays to infinitely small
    robot.wshm("plg_stiffness",       robot.stiffness)
    robot.wshm("plg_damping",         robot.damping)

    robot.controller(conf['fade_controller'])





def launch_mainloop():
    #mainloop()
    conf['thread']=Thread(target=mainloop)
    conf['thread'].start()





def get_selector_prop(pos):
    """ Map the joystick position to a position of a selector on the screen."""

    # Determine the joystick position on a range from 0 to 1
    joy = pos[0]
    joyrel = (joy-conf['min_joystick'])/(conf['max_joystick']-conf['min_joystick'])
    if joyrel<0: joyrel=0
    if joyrel>1: joyrel=1
    #print(joyrel)

    return joyrel




def update_selector(dpos):
    """ Given a mouse move event (change in x, change in y),
    update the selector value."""
    dchange = sum(dpos)*conf['mouse_selector_tick']
    p = trialdata['selector_prop']+dchange
    p = p%1
    trialdata['selector_prop']=p
    selector_to_angle()



def selector_to_angle():
    """ Map a value in the selector range (0..1) to an actual angle (in deg) """
    prop = trialdata.get('selector_prop',None)
    if prop:
        mn,mx = conf['arc_range']
        a = mn+(mx-mn)*prop
        trialdata['selector_angle']=a

        ## Add this to the history of selections
        hist = trialdata.get('selector_history',[])
        if len(hist)==0 or hist[-1]!=a:
            hist.append(a)
        trialdata['selector_history']=hist





def get_joystick():
    """ Get the current position of the joystick."""
    joystick = conf['joystick']
    pos = [ joystick.get_axis( i ) for i in range(2) ] # get the first two axes
    if 'joystick_history' in trialdata:
        if len(trialdata['joystick_history'])==0 or pos!=trialdata['joystick_history'][-1]:
            trialdata['joystick_history'].append(pos) # TODO: make sure we clear the joystick history also!
    trialdata['selector_prop']=get_selector_prop(pos) # update the selector position
    selector_to_angle()
    return pos





def current_schedule():
    # Return the item that is currently scheduled
    if 'current_schedule' not in trialdata or trialdata['current_schedule']==-1: return None
    return trialdata['schedule'][trialdata['current_schedule']]


def phase_is(p):
    # Return whether the current phase is p
    return trialdata.get('phase',None)==p

def phase_in(phases):
    # Return whether the current phase is p
    return trialdata.get('phase',None)in phases




def start_new_trial():
    """ Initiate a new trial. """
    # Okay, if that wasn't the case, we can safely start our new trial
    trialdata['current_schedule']+=1
    
    
    gui['progress']['maximum'] =len(trialdata['schedule'])
    gui['progress']['value']   =trialdata['current_schedule']
    #gui['progress'].update()

    if trialdata['current_schedule']==len(trialdata['schedule']):
        # Done the experiment!
        print("## BLOCK COMPLETED ##")
        robot.move_to(conf['robot_center_x'],conf['robot_center_y'],conf['return_duration'])
        while not robot.move_is_done():
            time.sleep(.1)
        robot.stay() # just fix the handle wherever it is
        next_phase('completed')
        gui['keep_going'] = False # this will bail out of the main loop
        gui['running'] = False
        time.sleep(1) # wait until some last commands may have stopped
        close_logs()
        update_ui()
        # Now ask the experimenter for observations
        #obsv = tksd.askstring('Please record any observations', 'Experimenter, please write down any observations.\nAny irregularities?\nDid the subject seem concentrated or not?\nWere things unclear or clear?\nAnything else that is worth noting?')
        #with open(conf['obsvlog'],'w') as f:
        #    f.write(obsv)

        gui['trialinfo'].set('Completed block')
        tkMessageBox.showinfo("Robot", "Block completed! Yay!")
        return



    sched = current_schedule() # Retrieve the current schedule
    #print(sched)
    trialdata['timestamp']       =time.time()
    trialdata['schedule.number'] =trialdata['current_schedule']
    trialdata['trial']           =sched['trial'] # trial number
    trialdata['type']            =sched['type']
    trialdata['target.direction']=sched['target.direction']  # display angle of the target
    trialdata['mov.direction']   =sched['mov.direction']     # physical angle of movement
    trialdata['movement_position']=None
    trialdata['cursor.rotation'] =sched['cursor.rotation']   # rotation applied to cursor before display
    trialdata['force.field']     =sched['force.field']       # the force field (if any)
    trialdata['captured']        =[] # nothing captured
    trialdata['review.shown']    =False
    #trialdata['position_history']=[] # start with a clean position history (we'll fill this up during active trials only)

    for v in ['vmax_x','vmax_y','final_x','final_y']:
        trialdata[v]=None


    trialinfo =  "trial %d %s   "%(trialdata['trial'],trialdata['type'])
    trialinfo += ("targ: %.2f   mov: %.2f   rot: %.2f deg;   ff: %s\n"%(trialdata['target.direction'],trialdata['mov.direction'],trialdata['cursor.rotation'],trialdata['force.field']))
    print('\n\n\n### TRIAL %d %s ####'%(trialdata['trial'],trialdata['type']))
    print(trialinfo)
    gui['trialinfo'].set(trialinfo)
    robot.wshm('fvv_trial_no',     trialdata['trial'])

    ## Return the robot to the center
    sx,sy = conf['robot_center']
    robot.move_to(sx,sy,conf['return_duration'])
    next_phase('return') # return to the starting point to start the trial





def record_position(x,y):
    # trialdata['position_history'].append((x,y)) # let's not do this -- we can use the info captured from the robot itself
    pass



def position_from_angle(a,radius=None):
    """ Return the position based on the angle, relative to the starting point
    and assuming a movement of a standard radius."""
    if radius==None:
        radius = conf['movement_radius']
    cx,cy = conf['robot_center']
    pos = (cx+radius*np.cos(a),cy+radius*np.sin(a))
    return pos



def start_move_controller(trialdata):
    """ Start the move phase """

    # The variables below are set so that we can determine when the movement ends, namely
    # by looking at the peak velocity profile.
    robot.wshm('fvv_robot_center_x',conf['robot_center_x'])
    robot.wshm('fvv_robot_center_y',conf['robot_center_y'])
    robot.wshm('fvv_move_done',0) # we'll set this to one once the movement is completed
    robot.wshm('fvv_min_dist',conf['min_movement_radius'])
    robot.wshm('fvv_max_vel',0)
    robot.wshm('fvv_vmax_x',0)
    robot.wshm('fvv_vmax_y',0)
    robot.wshm('fvv_vel_low_timer',0)
    robot.wshm('fvv_subject_move_phase',1) # signal that this is the subject move phase
    robot.wshm('traj_count',0)

    if trialdata['force.field']=='none':
        robot.controller(conf['move_controller'])

    if trialdata['force.field']=='channel':
        # This will be a channel trial between the start point and the target
        robot.wshm('plg_p1x',conf['robot_center_x'])
        robot.wshm('plg_p1y',conf['robot_center_y'])
        tx,ty= trialdata['target_position']
        robot.wshm('plg_p2x',tx)
        robot.wshm('plg_p2y',ty)
        robot.wshm("plg_channel_width", 0.0)
        robot.wshm("plg_stiffness", conf['channel_stiffness'])
        robot.wshm("plg_damping", conf['channel_damping'])
        robot.controller(15)

    if trialdata['force.field']=='curl':
        #print("Activating curl controller, curl=%.2f"%ffval)
        ffval = conf['force_field_curl']
        if ffval > 18: ffval = 18 # for safety
        robot.wshm("curl",ffval)
        robot.controller(17)





def draw_line(surf,fromp,top,col,width):
    """ Draw a line from a particular point to another point, in robot coordinates."""
    fromr = robot_to_screen(fromp[0],fromp[1],conf)
    tor   = robot_to_screen(top[0],  top[1]  ,conf)
    pygame.draw.line(surf,col,fromr,tor,width)




def review_plot():
    """ Make a plot of the trajectory we recorded from the robot."""

    if 'review.shown' in trialdata and trialdata['review.shown']:
        return # Review has already been shown for this trial

    #trajx,trajy = zip(*traj) # unzip!
    plot = pygame.Surface(conf['screensize'])
    plot.fill(conf['bgcolor'])

    # Draw start position
    draw_ball(plot,conf['robot_center'],conf['center_marker_radius'],conf['center_marker_colour'])
    # Draw target
    if 'target_position' in trialdata and not trialdata['type']=='pinpoint': # We show a target always, only not if this is a pinpoint trial
        col = get_target_colour()
        draw_ball(plot,trialdata['target_position'],conf['target_radius'],col)

    if trialdata['type']=='pinpoint':
        draw_arc_selector(plot)

    if 'movement_position' in trialdata and trialdata['movement_position']: # this is where the subject was moved to
        draw_ball(plot,trialdata['movement_position'],conf['cursor_radius'],conf['review_passive_colour'])
        if trialdata['type'] in ['passive','pinpoint']:
            draw_line(plot,conf['robot_center'],trialdata['movement_position'],conf['review_passive_colour'],conf['review_passive_width'])

        if not np.isinf(trialdata['cursor.rotation']): # shahryar - just this line plus ¨else¨ line is added (i.e. rotpos = rotate(...) was wxisting without any if clause)
            rotpos = rotate(trialdata['movement_position'],deg2rad(trialdata['cursor.rotation']),conf['robot_center'])
        else: #shahryar
            rotpos = 0 #shahryar

        draw_ball(plot,rotpos,conf['cursor_radius'],conf['passive_cursor_colour'])

        if trialdata['type'] in ['passive','pinpoint']:
            draw_line(plot,conf['robot_center'],rotpos,conf['passive_cursor_colour'],conf['review_passive_width'])


    if 'captured' in trialdata and len(trialdata['captured'])>0: # if there is actually something captured

        traj = [ (x,y,fx,fy,fz) for (x,y,fx,fy,fz) in trialdata['captured'] ] # make a copy, which seems to be a good idea here
        if len(traj)==0:
            print("Captured 0 samples")
            return
        # Just resample a little bit, so that the drawing will consume less resources

        #print(traj)
        traj = traj[ ::10 ]

        # Draw the forces also (how cool is that)
        for (x,y,fx,fy,_) in traj[::3]: # further subsampling also
            sx,sy = robot_to_screen(x,y,conf)
            tx,ty = robot_to_screen(x+conf['review_force_scale']*fx,
                                    y+conf['review_force_scale']*fy,
                                    conf)
            pygame.draw.line(plot,conf['review_force_colour'],(sx,sy),(tx,ty),conf['review_force_width'])

        # Draw the actual positions
        points = [ robot_to_screen(x,y,conf) for x,y,_,_,_ in traj ]
        if len(points):
            pygame.draw.lines(plot,conf['review_trajectory_colour'],False,points,conf['review_linewidth'])

        # Draw the corresponding cursor display
        if trialdata['cursor.rotation']!=0 and not np.isnan(trialdata['cursor.rotation']) and not np.isinf(trialdata['cursor.rotation']):
            rottraj = [ rotate((x,y),deg2rad(trialdata['cursor.rotation']),conf['robot_center']) for x,y,_,_,_ in traj ]
            points = [ robot_to_screen(x,y,conf) for (x,y) in rottraj ]
            if len(points):
                pygame.draw.lines(plot,conf['review_rotated_colour'],False,points,conf['review_linewidth'])

    show_review(plot,trialdata)




def show_review(plot,trialdata):
    """ Given a pygame surface, show it in the review window."""
    global gui
    fname = '.sneak_peek.bmp'
    jpg = '.tmp.jpg'
    gif = '.screenshot.gif'
    pygame.image.save(plot,fname)
    subprocess.call(['convert',fname,
                     '-crop','800x500+550+100',
                     '-flip',
                     '-resize','480x300',
                     '+repage',
                     gif])
    #subprocess.call(['convert',jpg,gif])
    trialdata['saved']=True
    gui["photo"]=PhotoImage(file=gif)
    gui["photolabel"].configure(image=gui["photo"])

    trialdata['review.shown'] = True # mark that we've shown the review for this trial, no need to do that again








def recognition_review(trialdata,hist):
    """ Make a plot of the trajectory we recorded from the robot."""

    plot = pygame.Surface(conf['screensize'])
    plot.fill(conf['bgcolor'])

    # Draw start position
    cx,cy = conf['robot_center']
    draw_ball(plot,(cx,cy),conf['center_marker_radius'],conf['center_marker_colour'])

    # For reference, draw straight ahead as well
    tpos =position_from_angle(conv_ang(0))
    draw_ball(plot,tpos,conf['center_marker_radius'],conf['center_marker_colour'])

    # Draw the two movement directions
    #colors = {'A':(255,0,0),'B':(0,255,0)}
    col = (255,255,0)
    pos = hist['direction.xy']
    points = [ robot_to_screen(x,y,conf) for (x,y) in [(cx,cy),pos]]
    pygame.draw.lines(plot,col,False,points,3)
    draw_ball(plot,pos,conf['target_radius'],col)

    show_review(plot,trialdata)



# shr
def get_target_sound():
    """ After a trial is completed, get the target sound."""
    xxmax = trialdata.get('final_x',None) #robot.rshm('fvv_max_vel')
    yymax = trialdata.get('final_y',None)

    if (not xxmax) or (not yymax): # if we don't have a reading
        return conf["target_slow_tone"]
    
    cx,cy = conf['robot_center']
    d = math.sqrt(pow(xxmax-cx, 2) + pow(yymax-cy, 2))
    if d<conf['movement_radius']:
        return conf["target_correct_tone"]
    if d>conf['movement_small_range']:
        return conf["target_fast_tone"]
    return conf["target_correct_tone"]
# shr




def get_target_colour():
    """ After a trial is completed, get the target colour."""
    vmax = trialdata.get('max_vel',None) #robot.rshm('fvv_max_vel')
    if not vmax: # if we don't have a reading
        return conf['target_colour']

    if vmax<conf['too_slow_vmax']:
        return conf['target_slow_colour']
    if vmax>conf['too_fast_vmax']:
        return conf['target_fast_colour']
    return conf['target_correct_colour']




def visual_error_clamp( pos, start, target):
    """
    Return the cursor display position in a visual error clamp trial.

    pos : (x,y) tuple denoting current position
    start : (x,y) tuple denoting the starting position
    target : (x,y) tuple denoting the target position
    """

    pos    = np.array(pos)
    start  = np.array(start)
    target = np.array(target)

    def d(a,b): return np.sqrt( sum(pow(a-b,2)) )

    # How far have we traveled from the starting point?
    dist = d(pos,start)

    # TODO: if we are moving *away* from the target, we will want to make dist negative
    if pos[1]<start[1]:
        dist=-dist
    #dpostarg   = d(pos,target)
    dstarttarg = d(start,target)
    #if dpostarg>dstarttarg:
    #    dist=-dist # count this as moving away

    # Unit vector pointing from target to start
    targvect = (target-start)/dstarttarg

    # Now take that amount and apply it to the vector start-to-target
    return start + dist*targvect , dist







def in_start_zone(trialdata):
    """ Tell us whether the robot handle is in the starting zone."""

    # Compute how far we are from the starting zone
    dstart = np.sqrt(pow(conf['robot_center_x']-trialdata['robot_x'],2)+pow(conf['robot_center_y']-trialdata['robot_y'],2))
    return dstart<.5*conf['center_marker_radius']





def mainloop():

    gui['running']=True
    gui['keep_going']=True # be an optimist!
    trialdata['redraw']=True
    trialdata['first.t']=time.time()
    pygame.event.clear() # Make sure there is no previous events in the pipeline
    if conf['use_mouse']:
        conf['mouse'].purgeEvents()

    while gui['keep_going']:

        time.sleep(.0001) # add a little breath
        trialdata["t.absolute"] = time.time()
        trialdata["t.current"] = trialdata["t.absolute"]-trialdata['first.t']
        schedule = current_schedule()


        ##
        ## INPUT PHASE
        ##

        # Read the current position
        trialdata['robot_x'],trialdata['robot_y'] = robot.rshm('x'),robot.rshm('y')
        if phase_in(['fade','move']):
            record_position(trialdata['robot_x'],trialdata['robot_y'])

        # Get any waiting events (joystick, but maybe other events as well)
        if conf['use_joystick']:
            evs = pygame.event.get()
            if phase_is('select'):
                for ev in evs:
                    if ev.type==pygame.JOYAXISMOTION:
                        trialdata['joystick.pos']=get_joystick() # update the joystick position
                        trialdata['redraw']=True
                    if ev.type==pygame.JOYBUTTONDOWN:
                        trialdata['selection_made']=True

        if conf['use_mouse']:
            ev = conf['mouse'].getEvent()
            if ev and phase_is('select'): # Update the selector position based on this movement
                x,y=ev[0],ev[1]
                update_selector((x,y))
                trialdata['redraw']=True

                for i in range(2,len(ev)-1):
                    if ev[i]: trialdata['selection_made']=True





        ##
        ## CONTROL FLOW
        ##
        if phase_is('init'):
            # Start a new trial
            start_new_trial()

        if phase_is('return'):
            if robot.move_is_done(): # if we are back at the starting point
                robot.stay()
                next_phase('pause')
                trialdata['pause.until.t']=trialdata['t.absolute']+conf['pause_duration']

        if phase_is('pause'):
            if trialdata['t.absolute']>trialdata.get('pause.until.t',0): # if the hold time is expired
                if schedule['type'] in ['passive','pinpoint','active']:

                    # Determine the angle of the display target
                    angle = conv_ang(schedule['target.direction'])
                    trialdata['target.display.angle']=angle
                    trialdata['target_position']=position_from_angle(angle)

                    if schedule['type'] in ['passive','pinpoint']:

                        angle = conv_ang(schedule['mov.direction'])
                        trialdata['movement_angle']=angle
                        radius = conf['movement_radius']
                        if schedule['type']=='passive': # In passive trials, we add a little undershoot/overshoot
                            o = np.random.normal(0,conf['target_overshoot_SD'])
                            trialdata['movement_overshoot']=o
                            radius+=o
                        trialdata['movement_position']=position_from_angle(angle,radius)
                        tx,ty = trialdata['movement_position']
                        robot.move_to(tx,ty,conf['passive_duration'])
                        next_phase('forward')

                    if schedule['type']=='active': # active movement
                        hold_fade()
                        robot.background_capture()
                        # the above line  will activate position capturing in the background,
                        # so that we can later pull the captured trajectory out
                        next_phase('fade')
                        trialdata['hold.until.t']=trialdata['t.absolute']+conf['fade_duration']

                        if DEBUG: # in the debug mode, we will simulate a subject's movement (because we are not connected to the physical robot)
                            x,y=trialdata['target_position']
                            robot.preprogram_move_to(x,y,1.5)
                            robot.future.append({"fvv_max_vel":.46}) # send a max_vel value for testing
                            robot.future.append({"fvv_move_done":1}) # this will happen in the future when the entire trajectory has finished

                    trialdata['redraw']=True

        if phase_is('fade'):
            if trialdata['t.absolute']>trialdata.get('hold.until.t',0): # if the hold time is expired
                subprocess.Popen(['aplay', conf["start_tone"]])
                start_move_controller(trialdata)
                next_phase('move')
                robot.wshm('fvv_trial_phase',5) # signal that we are moving
                trialdata['redraw']=True
                trialdata['t.move.start']=trialdata['t.absolute']

        if phase_is('move'):
            if robot.rshm('fvv_move_done'): # in active functioning mode
                #if trialdata['t.absolute']>trialdata['t.move.start']+2 : # DEBUG ONLY robot.rshm('fvv_move_done'): # this is the signal from the move controller that the subject has stopped moving
                robot.stay()
                capt = robot.stop_background_capture() # stop capturing and return what we have captured
                trialdata['captured'] = [ (x,y,fx,fy,fz) for (x,y,fx,fy,fz) in capt ] # make a copy, which seems to be a good idea here, otherwise I had issues when saving to pickle for example

                # Read the position at peak velocity
                for vr in ['vmax_x','vmax_y','final_x','final_y','max_vel']:
                    trialdata[vr]=robot.rshm('fvv_%s'%vr)

                next_phase('hold')
                trialdata['hold.until.t']=trialdata['t.absolute']+conf['stay_duration']

                # shr - play target sound
                subprocess.Popen(['aplay', get_target_sound()])

        if phase_is('hold'):
            if trialdata['t.absolute']>trialdata.get('hold.until.t',0):
                review_plot()
                answer = messagebox.askyesno("Question","Was the visual displacement to the Left (YES) or Right (NO) of your own movement?")
                trialdata['overshoot_answer']='left' if answer else 'right'
                next_phase('completed')
        
        
        
        if phase_is('forward'):
            if robot.move_is_done():
                if schedule['type'] in ['passive','pinpoint']:
                    robot.stay()
                    trialdata['stay.until.t']=trialdata['t.absolute']+conf['stay_duration']
                    next_phase('stay')

        if phase_is('stay'):
            if trialdata['t.absolute']>trialdata.get('stay.until.t',0):
                if schedule['type'] in ['passive','pinpoint']:
                    sx,sy = conf['robot_center']
                    robot.move_to(sx,sy,conf['passive_duration'])
                    next_phase('backward')

        if phase_is('backward'):
            if robot.move_is_done():
                if schedule['type'] in ['passive']:
                    robot.stay()
                    next_phase('ask')

                if schedule['type']=='pinpoint':
                    robot.stay()

                    r = random.random()
                    trialdata['selector_prop']   =r
                    trialdata['selector_initial']=r # mark that this was the first angle, in case we need to report this later on
                    trialdata['selection_made']=False
                    next_phase('select')



# =============================================================================
#         if phase_is('ask'):
#             r = numpy.random.uniform()
#             if r<conf['target_overshoot_ask_p']:
#                 review_plot()
#                 answer = messagebox.askyesno("Question","Did the cursor overshoot (YES) or undershoot (NO) the center of the target?")
#                 trialdata['overshoot_answer']='overshoot' if answer else 'undershoot'
#             next_phase('completed')
# =============================================================================


        if phase_is('select'):
            if trialdata['selection_made']:
                print("Selected angle %.2f deg (%.2f)"%(trialdata['selector_angle'],trialdata['selector_prop']))
                next_phase('completed') # this will automatically wrap up the trial


        if phase_is('completed'):
            review_plot()
            write_logs()
            start_new_trial()





        #
        #       DISPLAY
        # Possibly update the display
        #
        if trialdata['redraw']:

            conf['screen'].fill(conf['bgcolor'])

            clamp_trial    = np.isnan(trialdata['cursor.rotation']) # Decide whether this is an error clamp trial
            noexparc_trial = np.isinf(trialdata['cursor.rotation']) # shahryar - Decide whether this is an no-expanding-arc trial

            # Draw start position
            draw_ball(conf['screen'],conf['robot_center'],conf['center_marker_radius'],conf['center_marker_colour'])

            if 'target_position' in trialdata and not trialdata['type']=='pinpoint': # We show a target always, only not if this is a pinpoint trial

                # Determine the colour
                col = get_target_colour() if phase_is('hold') else conf['target_colour']


                # Now draw the target.
                # If we want to draw the target as an arc...
                if schedule and schedule.get('target.type',None)=='arc':
                    # if clamp_trial and conf['na.bar.show'] and conf['bar.as.arc']:
                    draw_arc(conf['screen'],conf['movement_radius'],col)
                elif schedule and schedule.get('target.type',None)=='point': #  or draw the target as a ball
                    draw_ball(conf['screen'],trialdata['target_position'],conf['target_radius'],col)

            if phase_is('select'): # If we are in the select phase
                if 'selector_prop' in trialdata: selector_to_angle()
                draw_arc_selector(conf['screen'])

            # For debug only: show veridical robot position
            if DEBUG:
                draw_ball(conf['screen'],(trialdata['robot_x'],trialdata['robot_y']),conf['cursor_radius'],(100,100,100)) # only for debug: show the real robot position

            if phase_in(['forward','stay','fade','move']):

                # Show a cursor
                if trialdata['type'] in ['passive','active']:

                    # Determine the colour for the cursor (point cursor or arc cursor)
                    if phase_is('fade'):
                        colour=conf['fade_cue_colour']
                    elif phase_is('move'):
                        colour = conf['active_cursor_colour']
                    else:
                        colour = conf['passive_cursor_colour']

                    if clamp_trial or noexparc_trial: # shahryar - changed
                        if conf['na.bar.show']:
                            trialdata['cursor_position']=(trialdata['robot_x'],trialdata['robot_y'])
                            trialdata['bar_position'],trialdata['bar_distance']=visual_error_clamp( (trialdata['robot_x'],trialdata['robot_y']),
                                                                                                    conf['robot_center'],
                                                                                                    trialdata['target_position'] )
                        else:
                            assert False # not implemented, ask me this another day

                    else:
                        # Rotate the cursor by a specified amount
                        trialdata['cursor_position']=rotate((trialdata['robot_x'],trialdata['robot_y']),
                                                            deg2rad(trialdata['cursor.rotation']),conf['robot_center'])
                    # OLD -- We show the cursor, but if the rotation is NA (i.e. no-feedback trial) then we only show it as long
                    # as it's in the target zone.
                    #if showcursor or ( (trialdata['type']=='active') and in_start_zone(trialdata)): # or (trialdata['type']=='active' and phase_is('fade')):
                    showcursor = True # not np.isnan(trialdata['cursor.rotation']) # the signal to hide the cursor is setting cursor.rotation to NA
                    if (not conf['na.cursor.show']) and (clamp_trial or noexparc_trial): # shahryar - changed
                        # Show the cursor only in the starting zone, hide it when it's outside
                        showcursor = (trialdata['type']=='active') and in_start_zone(trialdata)

                    # Decide whether we want to show a "horizontal" bar moving along with the subject...
                    if clamp_trial and conf['na.bar.show'] and not in_start_zone(trialdata):

                            if conf['bar.as.arc']:
                                draw_arc(conf['screen'],trialdata['bar_distance'],colour)
                            else:
                                _,cursor_y = trialdata['bar_position']
                                draw_bar(conf['screen'],cursor_y)

                    if showcursor:
                        # Always show the cursor
                        draw_ball(conf['screen'],trialdata['cursor_position'],conf['cursor_radius'],colour)





            pygame.display.flip()


    print("Bailed out of main loop.")
    gui['running']=False






def next_phase(p):
    """ Set the current phase to a given value. """
    print(p)
    trialdata['phase']=p

    # Track the start of this phase
    k = trialdata.get('t.phase',{})
    k[p]=time.time()
    trialdata['t.phase']=k

    # Mark on the robot that this phase started too
    if p in conf['phases']:
        robot.wshm('fvv_trial_phase',conf['phases'].index(p))
    else:
        print("## WARNING, unknown phase %s"%p)







#
#
#  Stuff relating to logging
#
#



def init_logs():
    #logfile = open("data/exampledata.txt",'w')

    basedir = './data/%s'%conf['participant']
    if not os.path.exists(basedir):
        os.makedirs(basedir)

    timestamp = datetime.datetime.now().strftime("%d_%m.%Hh%Mm%S")
    schedulestem = os.path.splitext(os.path.basename(conf['schedulefile']))[0]
    basename = '%s/%s_%s_%s--%s--'%(basedir,conf['participant'],EXPERIMENT,timestamp,schedulestem)

    trajlog = '%strajectory.bin'%basename
    robot.start_log(trajlog,conf['N_ROBOT_LOG'])
    gui["logging"]=True

    conf['captured'] = []
    #capturelog = '%scaptured.pickle27'%basename
    ##trajlog.write('participant experiment trial x y dx dy t t.absolute\n')

    dumplog = '%sdump.pickle'%basename

    ## Also dump the configuration parameters, this will make it easier to debug in the future
    conflog = open('%sparameters.json'%basename,'w')
    params = {}
    for key in sorted(conf):
        if key not in ['joystick','screen','mouse','thread','triallog']: # don't dump that stuff: not serialisable
            params[key]=conf[key]
    params['schedule']=trialdata['schedule']
    #print(params)
    json.dump(params,conflog)
    conflog.close()

    conf['obsvlog']     = '%sobservations.txt'%basename # where we will write the experimenter observations

    triallog = '%strials.txt'%basename
    conf['triallog'] = open(triallog,'w')
    conf['triallog'].write(trial_header())

    conf['trajlog']     = trajlog
    conf['dumpf']       = dumplog
    #conf['capturelog']  = capturelog





def write_logs():
    """ At the end of a trial, write into the log. """
    hist = {}
    for k in ['type','target.direction','mov.direction','cursor.rotation','target_position','cursor_position','t.phase','selector_angle','selector_prop','selector_history','selector_initial','captured','overshoot_answer','movement_overshoot','trial','schedule.number']:
        v = trialdata.get(k,np.nan)
        if isinstance(v,np.ndarray):
            v = v.tolist()
        hist[k]=v
    conf['trialhistory'].append(hist)
    pickle.dump(conf['trialhistory'],open(conf['dumpf'],'wb')) # this will overwrite the previous file
    #h5f = h5py.File(conf['dumpf'], 'w')
    #for h in conf['trialhistory']:
    #    h5f.create_dataset(h['trial'], data=h)
    #h5f.close()

    conf['triallog'].write(trial_log())
    conf['triallog'].flush()



LOG_COLUMNS = ['trial','type','mov.direction','cursor.rotation','selector_angle','selector_prop','max_vel','vmax_x','vmax_y','final_x','final_y','movement_overshoot','overshoot_answer','timestamp']
def trial_header():
    return " ".join(LOG_COLUMNS)+"\n"

def trial_log():
    return " ".join([str(trialdata.get(v,np.nan)) for v in LOG_COLUMNS])+"\n"



def close_logs():
    if gui["logging"]:
        if 'triallog' in conf:
            conf['triallog'].close()
        robot.stop_log()
    gui["logging"]=False





def read_schedule_file():
    """
    Read a schedule file which tells us the parameters of each trial.
    """

    print("Reading trial schedule file %s"%conf['schedulefile'])

    s = pd.read_csv(conf['schedulefile'],sep=',')
    for c in ['trial','type','target.direction','mov.direction','cursor.rotation','force.field','target.type']:
        if not c in s.columns:
            print("## ERROR: missing column %s in schedule."%c)
            return False

    if not is_string_dtype(s['force.field']):
        print("## ERROR: force.field column has to be a string.")
        return False

    ttypes = list(set(s['target.type']))
    for tt in ttypes:
        if tt not in ['arc','point', 'none']:
            print("## ERROR: unknown target type {}, has to be arc, point, or none even according to Indian standards.".format(tt))
            return False


    # I don't trust pandas so I prefer a simple data structure, list of dicts
    schedule = []
    for i,row in s.iterrows():
        row = dict(row)
        #print(row['force.field'])
        row['force.field']=row['force.field'].strip().lower()
        if not row['force.field'] in ['none','curl','channel']:
            print("## ERROR: unknown force field value %s"%row['force.field'])
            return False
        schedule.append(dict(row))

    trialdata['schedule'] = schedule
    print("Finished reading %i trials"%(len(trialdata['schedule'])))
    return True






def update_ui():
    global gui
    gui["runb"].configure(state=DISABLED)
    gui["recogb"].configure(state=DISABLED)
    gui["mcb"].configure(state=DISABLED)
    gui["capturecenter"].configure(state=DISABLED)
    gui["holdcenter"].configure(state=DISABLED)

    if gui["loaded"]:
        gui["loadb"].configure(state=DISABLED)

        if not gui["running"]:
            gui["runb"].configure(state=NORMAL)
            gui["recogb"].configure(state=NORMAL)
            gui["mcb"].configure(state=NORMAL)
            gui["capturecenter"].configure(state=NORMAL)
            gui["holdcenter"].configure(state=NORMAL)

    else: # not loaded
        gui["loadb"].configure(state=NORMAL)




def load_robot():
    """ Launches the robot process. """
    global gui
    tkMessageBox.showinfo("Robot", "We will now load the robot.\n\nLook at the terminal because you may have to enter your sudo password there.")

    robot.load() # launches the robot C++ script
    if DEBUG: robot.popup(master)
    # Then do zero FT
    tkMessageBox.showinfo("Robot", "Now we will zero the force transducers.\nAsk the subject to let go of the handle." )
    robot.zeroft()
    bias = robot.bias_report()
    global trialdata
    trialdata['bias']=bias
    tkMessageBox.showinfo("Robot", "Robot FT bias calibration done.\nYou can tell the subject to hold the handle again.\n\nRobot bias settings:\n\n" +" ".join([ str(b) for b in bias ]))
    gui["loaded"]=True
    update_ui()






def stop_running():
    print("Currently {} threads still active.".format(threading.active_count()))
    gui['keep_going'] = False # this will bail out of the main loop
    time.sleep(1) # wait until some last commands may have stopped
    close_logs()
    if 'thread' in conf and conf['thread']:
        #conf['thread'].stop()
        pass # cannot kill thread unfortunately...

    if gui['loaded']:
        robot.unload()
        gui["loaded"]=False



def end_program():
    """ When the user presses the quit button. """
    stop_running()
    ending()
    master.destroy()
    print("Just before killing, {} thread(s) still active.".format(threading.active_count()))
    sys.exit(0)


def runrecog():
    """ Run the recognition task"""
    if gui["running"]:
        return

    global conf

    participant=gui["subject.id"].get().strip()
    if participant=="":
        tkMessageBox.showinfo("Error", "You need to enter a participant ID.")
        return
    conf['participant'] = participant

    # Ask to open a schedule file
    fn = filedialog.askopenfilename(filetypes = (("CSV files", "*.csv")
                                                 ,("All files", "*.*") ))
    if not fn or not len(fn):
        print("No file selected.")
        return
    schedulef = fn

    print("Reading schedule file {}.".format(schedulef))
    # Read the recognition script
    s = pd.read_csv(schedulef,sep=',')
    for c in ['trial','direction','type','target.direction']:
        if not c in s.columns:
            print("## ERROR: missing column %s in schedule file - is this really a yes/no recognition task schedule file?"%c)
            return False

    schedule = []
    for i,row in s.iterrows():
        row = dict(row)
        schedule.append(row)
    trialdata['schedule']=schedule

    # Read the robot center
    if not get_center(): return
    robot.wshm('fvv_robot_center_x',conf['robot_center_x'])
    robot.wshm('fvv_robot_center_y',conf['robot_center_y'])

    # Let's go!
    print("Launching recognition yes/no test: {} trials.".format(len(schedule)))

    # TODO: empty display
    conf['screen'].fill(conf['bgcolor'])
    pygame.display.flip()

    gui["running"]=True
    update_ui()

    conf['thread']=Thread(target=recognitiontest)
    conf['thread'].start()



class NpEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        else:
            return super(NpEncoder, self).default(obj)



def runmotorcopy():
    """ Run the motor copy version of the recognition task """
    if gui["running"]:
        return

    global conf

    participant=gui["subject.id"].get().strip()
    if participant=="":
        tkMessageBox.showinfo("Error", "You need to enter a participant ID.")
        return
    conf['participant'] = participant

    # Ask to open a schedule file
    fn = filedialog.askopenfilename(filetypes = (("CSV files", "*.csv")
                                                 ,("All files", "*.*") ))
    if not fn or not len(fn):
        print("No file selected.")
        return
    schedulef = fn

    print("Reading schedule file {}.".format(schedulef))
    # Read the recognition script
    s = pd.read_csv(schedulef,sep=',')
    for c in ['trial','direction','type','target.direction']:
        if not c in s.columns:
            print("## ERROR: missing column %s in schedule file - is this really a yes/no recognition task schedule file?"%c)
            return False

    schedule = []
    for i,row in s.iterrows():
        row = dict(row)
        row['type']=str(row['type'])
        schedule.append(row)
    trialdata['schedule']=schedule

    # Read the robot center
    if not get_center(): return
    robot.wshm('fvv_robot_center_x',conf['robot_center_x'])
    robot.wshm('fvv_robot_center_y',conf['robot_center_y'])

    # Initialise log information
    basedir = './data/%s'%conf['participant']
    if not os.path.exists(basedir):
        os.makedirs(basedir)
    timestamp = datetime.datetime.now().strftime("%d_%m.%Hh%Mm%S")
    basename = '%s/%s_%s_%s--motorcopy--'%(basedir,conf['participant'],EXPERIMENT,timestamp)
    jsonf    = basename+"parameters.json"   # where we will save the data
    capturef = basename+"dump.pickle"       # where we will save the data
    conf['basename']       = basename
    conf['jsonf']          = jsonf
    conf['capturef']       = capturef
    conf['trialhistory']   = []
    conf['capturehistory'] = []
    motor_copy_json_log()
    motor_copy_capture_log()

    trialdata['current_schedule']=-1 # start at the beginning
    trialdata['phase']='init'

    gui['progress']['maximum'] =len(trialdata['schedule'])
    gui['progress']['value']   =0


    # Open the logs
    trajlog = '%strajectory.bin'%basename
    robot.start_log(trajlog,conf['N_ROBOT_LOG'])
    gui["logging"]=True


    # Let's go!
    print("Launching motor copy test: {} trials.".format(len(schedule)))

    conf['screen'].fill(conf['bgcolor'])
    pygame.display.flip()

    gui["running"]=True
    update_ui()

    conf['thread']=Thread(target=mainloopmotorcopy)
    conf['thread'].start()





def motor_copy_json_log():
    # Write the JSON log for the motor copy test
    dat = {
            'robot_center':(conf['robot_center_x'],conf['robot_center_y']),
            'schedule':conf['trialhistory']
        }
    #print(dat)
    with open(conf['jsonf'],'w') as f:
        json.dump(dat,f,cls=NpEncoder)



def motor_copy_capture_log():
    # Write the (pickle) log for the motor copy test, the log that tracks the captured data
    with open(conf['capturef'],'wb') as f:
        pickle.dump(conf['capturehistory'],f)




def start_motor_copy_trial():

    trialdata['current_schedule']+=1

    gui['progress']['maximum'] =len(trialdata['schedule'])
    gui['progress']['value']   =trialdata['current_schedule']

    if trialdata['current_schedule']==len(trialdata['schedule']):
        # Done the experiment!
        print("## MOTOR COPY COMPLETED ##")
        robot.move_to(conf['robot_center_x'],conf['robot_center_y'],conf['return_duration'])
        while not robot.move_is_done():
            time.sleep(.1)
        robot.stay() # just fix the handle wherever it is
        next_phase('completed')
        gui['keep_going'] = False # this will bail out of the main loop
        gui['running'] = False
        time.sleep(1) # wait until some last commands may have stopped
        robot.stop_log()
        gui["logging"]=False
        update_ui()
        # Now ask the experimenter for observations
        #obsv = tksd.askstring('Please record any observations', 'Experimenter, please write down any observations.\nAny irregularities?\nDid the subject seem concentrated or not?\nWere things unclear or clear?\nAnything else that is worth noting?')
        #with open(conf['obsvlog'],'w') as f:
        #    f.write(obsv)

        gui['trialinfo'].set('Completed block')
        tkMessageBox.showinfo("Robot", "Motor copy completed! Yay!")
        return

    sched = current_schedule() # Retrieve the current schedule
    trialdata['timestamp']       =time.time()
    trialdata['schedule.number'] =trialdata['current_schedule']
    trialdata['trial']           =sched['trial'] # trial number
    trialdata['type']            =sched['type']
    trialdata['movement.direction']=sched['direction']         # where subjects will be moved passivly
    trialdata['movement.position'] =position_from_angle(conv_ang(sched['direction'])) # where subjects will be moved passively
    trialdata['target.direction']=sched['target.direction']  # display angle of the target
    trialdata['target.position'] =position_from_angle(conv_ang(sched['target.direction']))  # display angle of the target
    trialdata['movement_position']=None
    trialdata['force.field']     ='none'
    trialdata['captured']        =[] # nothing captured
    trialdata['review.shown']    =False
    #trialdata['position_history']=[] # start with a clean position history (we'll fill this up during active trials only)

    for v in ['vmax_x','vmax_y','final_x','final_y']:
        trialdata[v]=None

    #trialinfo =  "trial %d %s   "%(trialdata['trial'],trialdata['type'])
    trialinfo = ("targ: %.2f \n"%(trialdata['movement.direction']))
    print('\n\n\n### MOTOR COPY TRIAL %d TYPE %s ####'%(trialdata['trial'],trialdata['type']))
    print(trialinfo)
    gui['trialinfo'].set(trialinfo)
    robot.wshm('fvv_trial_no',     trialdata['trial'])

    ## Return the robot to the center
    sx,sy = conf['robot_center']
    robot.move_to(sx,sy,conf['return_duration'])
    next_phase('return') # return to the starting point to start the trial




def mainloopmotorcopy():

    gui['running']=True
    gui['keep_going']=True # be an optimist!

    trialdata['redraw']=True
    trialdata['first.t']=time.time()
    pygame.event.clear() # Make sure there is no previous events in the pipeline

    while gui['keep_going']:

        time.sleep(.0001) # add a little breath
        trialdata["t.absolute"] = time.time()
        trialdata["t.current"] = trialdata["t.absolute"]-trialdata['first.t']
        schedule = current_schedule()


        ##
        ## INPUT PHASE
        ##

        # Read the current position
        trialdata['robot_x'],trialdata['robot_y'] = robot.rshm('x'),robot.rshm('y')
        if phase_in(['fade','move']):
            record_position(trialdata['robot_x'],trialdata['robot_y'])


        ##
        ## CONTROL FLOW
        ##
        if phase_is('init'):
            # Start a new trial
            start_motor_copy_trial()

        if phase_is('return'):
            if robot.move_is_done(): # if we are back at the starting point
                robot.stay()
                next_phase('pause')
                trialdata['pause.until.t']=trialdata['t.absolute']+conf['recog_pause_duration']

        if phase_is('pause'):
            if trialdata['t.absolute']>trialdata.get('pause.until.t',0): # if the hold time is expired

                # Now start a forward movement to the given target direction
                angle = conv_ang(schedule['direction'])
                trialdata['movement_angle']=angle
                radius = conf['movement_radius']
                trialdata['movement_position']=position_from_angle(angle,radius)
                tx,ty = trialdata['movement_position']
                robot.move_to(tx,ty,conf['passive_duration'])
                next_phase('forward')

        if phase_is('forward'):
            if robot.move_is_done():
                robot.stay()
                trialdata['stay.until.t']=trialdata['t.absolute']+conf['stay_duration']
                next_phase('stay')

        if phase_is('stay'):
            if trialdata['t.absolute']>trialdata.get('stay.until.t',0):
                sx,sy = conf['robot_center']
                robot.move_to(sx,sy,conf['passive_duration'])
                next_phase('backward')

        if phase_is('backward'):
            if robot.move_is_done():

                hold_fade()
                robot.background_capture()
                # the above line  will activate position capturing in the background,
                # so that we can later pull the captured trajectory out
                next_phase('fade')
                trialdata['hold.until.t']=trialdata['t.absolute']+conf['fade_duration']

                if DEBUG: # in the debug mode, we will simulate a subject's movement (because we are not connected to the physical robot)
                    x,y=trialdata['target.position']
                    robot.preprogram_move_to(x,y,1.5)
                    robot.future.append({"fvv_max_vel":.46}) # send a max_vel value for testing
                    robot.future.append({"fvv_move_done":1}) # this will happen in the future when the entire trajectory has finished

                trialdata['redraw']=True

        if phase_is('fade'):
            if trialdata['t.absolute']>trialdata.get('hold.until.t',0): # if the hold time is expired
                start_move_controller(trialdata) ### TODO: change this somehow, this should probably not refer to this normal loop function
                next_phase('move')
                robot.wshm('fvv_trial_phase',5) # signal that we are moving
                trialdata['redraw']=True
                trialdata['t.move.start']=trialdata['t.absolute']

        if phase_is('move'):
            if robot.rshm('fvv_move_done'): # in active functioning mode
                #if trialdata['t.absolute']>trialdata['t.move.start']+2 : # DEBUG ONLY robot.rshm('fvv_move_done'): # this is the signal from the move controller that the subject has stopped moving
                robot.stay()
                capt = robot.stop_background_capture() # stop capturing and return what we have captured
                trialdata['captured'] = [ (x,y,fx,fy,fz) for (x,y,fx,fy,fz) in capt ] # make a copy, which seems to be a good idea here, otherwise I had issues when saving to pickle for example
                
                
                
                # Read the position at peak velocity
                for vr in ['vmax_x','vmax_y','final_x','final_y','max_vel']:
                    trialdata[vr]=robot.rshm('fvv_%s'%vr)

                next_phase('hold')
                trialdata['hold.until.t']=trialdata['t.absolute']+conf['stay_duration']

        if phase_is('hold'):
            if trialdata['t.absolute']>trialdata.get('hold.until.t',0):
                next_phase('completed')


        if phase_is('completed'):
            motorcopy_review_plot()

            # Write the log
            thistrial = {}
            for col in ['trial','movement.direction','movement.position','target.direction','target.position','type','timestamp','vmax_x','vmax_y','final_x','final_y']:
                thistrial[col] = trialdata[col]
            conf['trialhistory'].append(thistrial)
            motor_copy_json_log()

            # Write the captured trajectory log
            capt = {'trial'     :trialdata['trial'],
                    'timestamp' :trialdata['timestamp'],
                    'direction' :trialdata['movement.direction'],
                    'captured'  :list(trialdata['captured'])}
            conf['capturehistory'].append(capt)
            motor_copy_capture_log()

            ## write_logs() TODO this has to be done but for motor copy

            ## launch the next trial
            start_motor_copy_trial()



        #
        #       DISPLAY
        # Possibly update the display
        #
        if trialdata['redraw']:

            conf['screen'].fill(conf['bgcolor'])

            # Draw start position
            draw_ball(conf['screen'],conf['robot_center'],conf['center_marker_radius'],conf['center_marker_colour'])

            # Draw the target
            ### Determine the colour
            col = get_target_colour() if phase_is('hold') else conf['target_colour']

            ### Now draw the target.
            ### If we want to draw the target as an arc...
            draw_arc(conf['screen'],conf['movement_radius'],col)

            # For debug only: show veridical robot position
            if DEBUG:
                draw_ball(conf['screen'],(trialdata['robot_x'],trialdata['robot_y']),conf['cursor_radius'],(100,100,100)) # only for debug: show the real robot position

            if phase_in(['forward','stay','fade','move']):

                # Show a cursor

                # Determine the colour for the cursor (point cursor or arc cursor)
                if phase_is('forward') or phase_is('stay'):
                    colour=conf['motorcopy_forward_colour']
                elif phase_is('fade'):
                    colour=conf['fade_cue_colour']
                elif phase_is('move'):
                    colour = conf['active_cursor_colour']


                # Determine the cursor position
                trialdata['cursor_position']=(trialdata['robot_x'],trialdata['robot_y'])
                _,trialdata['bar_distance']=visual_error_clamp( (trialdata['robot_x'],trialdata['robot_y']),
                                                                conf['robot_center'],
                                                                trialdata['target.position'] )

                # Now actually draw the cursor
                cursor_is_arc = not in_start_zone(trialdata)
                in_start_zone(trialdata)

                if cursor_is_arc:
                    draw_arc(conf['screen'],trialdata['bar_distance'],colour)
                else:
                    # Always show the cursor
                    draw_ball(conf['screen'],trialdata['cursor_position'],conf['cursor_radius'],colour)

            pygame.display.flip()


    print("Bailed out of main motor copy loop.")
    gui['running']=False









def motorcopy_review_plot():
    """ Make a plot of the passive movement and following active movement."""

    plot = pygame.Surface(conf['screensize'])
    plot.fill(conf['bgcolor'])

    # Draw start position
    cx,cy = conf['robot_center']
    draw_ball(plot,(cx,cy),conf['center_marker_radius'],conf['center_marker_colour'])

    # For reference, draw straight ahead as well
    tpos =position_from_angle(conv_ang(0))
    draw_ball(plot,tpos,conf['center_marker_radius'],conf['center_marker_colour'])

    # Draw the movement direction
    #colors = {'A':(255,0,0),'B':(0,255,0)}
    col = (255,255,0)
    pos = trialdata['movement.position']
    points = [ robot_to_screen(x,y,conf) for (x,y) in [(cx,cy),pos]]
    pygame.draw.lines(plot,col,False,points,3)
    draw_ball(plot,pos,conf['target_radius'],col)

    # Draw the subject movement
    if 'captured' in trialdata and len(trialdata['captured'])>0: # if there is actually something captured

        traj = [ (x,y,fx,fy,fz) for (x,y,fx,fy,fz) in trialdata['captured'] ] # make a copy, which seems to be a good idea here
        if len(traj)==0:
            print("Captured 0 samples")
            return
        # Just resample a little bit, so that the drawing will consume less resources

        #print(traj)
        traj = traj[ ::10 ]

        # Draw the forces also (how cool is that)
        for (x,y,fx,fy,_) in traj[::3]: # further subsampling also
            sx,sy = robot_to_screen(x,y,conf)
            tx,ty = robot_to_screen(x+conf['review_force_scale']*fx,
                                    y+conf['review_force_scale']*fy,
                                    conf)
            pygame.draw.line(plot,conf['review_force_colour'],(sx,sy),(tx,ty),conf['review_force_width'])

        # Draw the actual positions
        points = [ robot_to_screen(x,y,conf) for x,y,_,_,_ in traj ]
        if len(points):
            pygame.draw.lines(plot,conf['review_trajectory_colour'],False,points,conf['review_linewidth'])

    show_review(plot,trialdata)








def get_center():
    """ Get the center X coordinate from the text entry box."""
    centerx=gui["centerv"].get().strip()
    try:
        centerx=float(centerx)
    except:
        tkMessageBox.showinfo("Error", "The center X location you entered is invalid: %s.\n\nIt has to be a floating number."%centerx)
        return
    conf['robot_center_x'] = centerx
    conf['robot_center']=(conf['robot_center_x'],conf['robot_center_y'])
    return True



def move_until_done(x,y,t):
    robot.move_to(x,y,t)
    while not robot.move_is_done():
        time.sleep(.1)
    return


def askoptions(message,options):
    """ Display a window that asks for options """
    win = Toplevel(master)
    win.title('Question')
    Label(win, text=message).pack()
    global selected
    selected = None
    def finish(option):
        global selected
        print('selected {}'.format(option))
        selected = option
    for option in options:
        Button(win, text=option, command=lambda option=option: finish(option)).pack()
    while not selected:
        time.sleep(.5)
    win.destroy()
    return selected






def recognition_display(trialdata,markersonly=False):
    # Draw the subject display screen for use during the recognition task
    conf['screen'].fill(conf['bgcolor'])

    trialdata['robot_x']=robot.rshm('x')
    trialdata['robot_y']=robot.rshm('y')

    # Draw start position
    draw_ball(conf['screen'],conf['robot_center'],conf['center_marker_radius'],conf['center_marker_colour'])

    # Draw the target (either arc or point)
    if conf['recognition_display_arc']:
        draw_arc(conf['screen'],conf['movement_radius'],conf['target_colour'])
    else:
        draw_ball(conf['screen'],trialdata['target_position'],conf['target_radius'],conf['target_colour'])

    # Draw a cursor
    if not markersonly: # if we also show a marker indicating the subject position somehow (could be a bar or arc or cursor)
        trialdata['bar_position'],dist=visual_error_clamp( (trialdata['robot_x'],trialdata['robot_y']),
                                                           conf['robot_center'],
                                                           trialdata['target_position'] )
        if conf['recognition_display_arc']:
            draw_arc(conf['screen'],dist,conf['active_cursor_colour'])
        else: # this stuff below is so obscene we don't do it anymore
            _,cursor_y = trialdata['bar_position']
            draw_bar(conf['screen'],cursor_y)

    pygame.display.flip()



def move_and_display(targetx,targety,movet,displayfunction):
    # Show a display while moving to a given location in a given time
    robot.move_to(targetx,targety,movet)
    while not robot.move_is_done():
        displayfunction()
        time.sleep(.001)



def recognitiontest():
    """ The main procedure for the recognition test."""
    gui['running']=True
    gui['keep_going']=True # be an optimist!

    t0 = time.time()

    # Initialise log information
    basedir = './data/%s'%conf['participant']
    if not os.path.exists(basedir):
        os.makedirs(basedir)
    timestamp = datetime.datetime.now().strftime("%d_%m.%Hh%Mm%S")
    basename = '%s/%s_%s_%s'%(basedir,conf['participant'],EXPERIMENT,timestamp)
    jsonf = basename+"recognition_yesno.json" # where we will save the data

    cx,cy = conf['robot_center_x'],conf['robot_center_y']
    move_until_done(cx,cy,conf['return_duration'])
    robot.stay_at(cx,cy)
    time.sleep(conf['recog_pause_duration'])

    showvis = bool(gui['recognitionvisuals'])

    current_schedule = 0
    history = []
    gui['progress']['maximum'] =len(trialdata['schedule'])
    for i,schedule in enumerate(trialdata['schedule']):
        gui['progress']['value']   =i
        gui['progress'].update()
        trialdata['trial']=schedule['trial']
        trialdata['target_position'] = position_from_angle(conv_ang(schedule['target.direction']),conf['movement_radius']) # this is where we will display the target

        hist = {'trial':schedule['trial'],'start.t':time.time()-t0}
        print("\n\n### Trial {} ###".format(trialdata['trial']))

        angle_deg = schedule['direction'] # this is where we will move the hand to
        angle = conv_ang(angle_deg) # Compute conventional angle
        tx,ty = position_from_angle(angle,conf['movement_radius'])
        print("Moving to direction : {:.2f} deg -> ({:.2f},{:.2f})".format(angle_deg,tx,ty))
        trialdata['movement']=tx,ty
        move_and_display(tx,ty,conf['passive_duration'], lambda : recognition_display(trialdata)) # move out to that direction
        robot.stay_at(tx,ty)
        time.sleep(conf['stay_duration'])
        recognition_display(trialdata,True)
        move_until_done(cx,cy,conf['return_duration']) # return to the center
        robot.stay_at(cx,cy)
        #time.sleep(conf['recog_pause_duration'])
        hist['direction'    ]     =angle_deg
        hist['direction_rad']     =angle
        hist['display_direction'] =schedule['target.direction']
        hist['display_position']  =trialdata['target_position']
        hist['type'         ]     =schedule['type']
        hist['direction.xy' ]     =tx,ty

        recognition_review(trialdata,hist)
        answer = askoptions("Was this the movement you produced?",['yes','no'])
        hist['answer']=answer #'A' if answer=='first' else 'B'
        hist['end.t']=time.time()-t0
        history.append(hist)
        savejson(history,jsonf)

        if not gui['keep_going']: return # This is if the user has pressed exit somehow

    print(history)
    savejson(history,jsonf)
    print("Results written to {}.".format(jsonf))
    print ("Completed recognition test.")

    gui['running']=False
    update_ui()



def savejson(history,jsonf):
    """ Write the history of the recognition task
    to the json file."""
    def default(o): # A hack so that we can save int64
        if isinstance(o, numpy.int64): return int(o)
        raise TypeError
    with open(jsonf,'w') as f:
        json.dump(history,f,default=default)




def run():
    """ Do one run of the experiment. """
    if gui["running"]:
        return

    global conf

    participant=gui["subject.id"].get().strip()
    if participant=="":
        tkMessageBox.showinfo("Error", "You need to enter a participant ID.")
        return
    conf['participant'] = participant


    schedulefile=gui["schedulef"].get().strip()
    if schedulefile=="":
        tkMessageBox.showinfo("Error", "You need to enter a schedule file name.")
        return
    if not os.path.exists(schedulefile):
        tkMessageBox.showinfo("Error", "The schedule file name you entered does not exist.")
        return
    conf['schedulefile'] = schedulefile


    centerx=gui["centerv"].get().strip()
    try:
        centerx=float(centerx)
    except:
        tkMessageBox.showinfo("Error", "The center X location you entered is invalid: %s.\n\nIt has to be a floating number."%centerx)
        return
    conf['robot_center_x'] = centerx
    conf['robot_center']=(conf['robot_center_x'],conf['robot_center_y'])
    robot.wshm('fvv_robot_center_x',conf['robot_center_x'])
    robot.wshm('fvv_robot_center_y',conf['robot_center_y'])


    if not read_schedule_file():
        tkMessageBox.showinfo("Error", "Error reading the schedule file. Please look at the terminal for the error message.")
        return

    print("Running the experiment.")
    gui["running"]=True
    update_ui()



    conf['trialhistory']=[] # this is where we will keep info from all previous trials.

    trialdata['participant'] =conf['participant']
    trialdata['schedulefile']=conf['schedulefile']
    init_logs()

    trialdata['current_schedule']=-1 # start at the beginning
    trialdata['phase']='init'

    launch_mainloop()




def select_schedule():
    """ Show a dialog that lets the user select a file. """

    fn = filedialog.askopenfilename(filetypes = (("CSV files", "*.csv")
                                                 ,("All files", "*.*") ))
    if not fn or not len(fn):
        print("No file selected.")
    else:
        gui["schedulef"].set(fn)







def capture_center():
    """ Capture the center position from the robot."""
    xs = []
    for i in range(conf['n_center_capture']):
        xs.append(robot.rshm('x'))
        time.sleep(conf['center_capture_period'])
    #print(xs)
    mean_x = np.mean(xs)
    gui['centerv'].set("%.3f"%mean_x)



def hold_center():
    """ Hold the robot handle at the center position."""
    if not get_center(): return
    cx,cy = conf['robot_center_x'],conf['robot_center_y']
    print("Holding at {},{}".format(cx,cy))
    robot.move_to(cx,cy,conf['return_duration'])
    while not robot.move_is_done():
        time.sleep(.1)
    robot.stay_at(cx,cy)
    print("Holding indefinitely.")
    return




def init_tk():
    global gui
    gui = {}

    master = Tk()
    master.geometry('%dx%d+%d+%d' % (CONTROL_WIDTH, CONTROL_HEIGHT, CONTROL_X, CONTROL_Y))
    master.configure(background='black')

    f = Frame(master,background='black')
    loadb   = Button(f, text="Load robot",                background="green",foreground="black", command=load_robot)
    runb    = Button(f, text="Run",                       background="blue", foreground="white", command=run)
    recogb  = Button(f, text="Recognition yes/no",        background="purple",foreground="white", command=runrecog)
    mcb     = Button(f, text="Motor copy",                background="orange",foreground="black", command=runmotorcopy)
    quitb   = Button(f, text="Quit",                      background="red",foreground="black", command=end_program)

    gui["subject.id"] = StringVar()
    l      = Label(f, text="subject ID",             fg="white", bg="black")
    subjid = Entry(f, textvariable=gui["subject.id"],fg="white", bg="black",insertbackground='yellow')

    row  = 0
    f.grid         (row=row,padx=10,pady=10)
    row += 1
    loadb.grid     (row=row,column=0,sticky=W,padx=10,pady=10)

    row += 1
    l.grid         (row=row,column=0,sticky=W,pady=10)
    subjid.grid    (row=row,column=1,sticky=W,padx=10)

    row +=1
    gui["schedulef"]  = StringVar()
    gui["schedulef"].set("schedule.csv")
    l      = Label(f, text="schedule file",          fg="white", bg="black")
    e      = Entry(f, textvariable=gui["schedulef"], fg="white", bg="black",insertbackground='yellow')
    l.grid(row=row,column=0,sticky=W,pady=10)
    e.grid(row=row,column=1,sticky=W,padx=10)
    b   = Button(f, text="select",                background="gray",foreground="black", command=select_schedule)
    b.grid(row=row,column=2,sticky=W,padx=10)

    row +=1
    gui["centerv"]  = StringVar()
    gui["centerv"].set("0.0")
    l      = Label(f, text="center X",             fg="white", bg="black")
    e      = Entry(f, textvariable=gui["centerv"], fg="white", bg="black",insertbackground='yellow')
    l.grid(row=row,column=0,sticky=W,pady=10)
    e.grid(row=row,column=1,sticky=W,padx=10)
    b   = Button(f, text="capture",                background="gray",foreground="black", command=capture_center)
    b.grid(row=row,column=2,sticky=W,padx=10)
    gui['capturecenter']=b
    b   = Button(f, text="hold",                   background="gray",foreground="black", command=hold_center)
    b.grid(row=row,column=3,sticky=W,padx=10)
    gui['holdcenter']=b

    row += 1
    runb.grid      (row=row,column=0,sticky=W,padx=10)
    recogb.grid    (row=row,column=1,padx=10)
    mcb.grid       (row=row,column=2,padx=10)

    b   = Button(f, text="pinpoint",             background="purple",foreground="black", command=pinpoint)
    #b.grid(row=row,column=1,sticky=W,padx=10)

    row += 1
    gui['recognitionvisuals']=IntVar()
    gui['recognitionvisuals'].set(1)
    #c = Checkbutton(f, text="Show visuals during recogn.", variable=gui['recognitionvisuals'], background='black',foreground='green',highlightcolor='black',highlightthickness=0)
    #c.grid(row=row,column=1)
    #Label(f,text="")
    quitb.grid     (row=row,sticky=W,padx=10,pady=10)

    row +=1
    l = Label(f,text='Progress',fg='white',bg='black')
    l.grid(row=row,column=0)
    p = ttk.Progressbar(f, orient=HORIZONTAL,mode='determinate',value=0,maximum=100)
    p['value']=0
    p.grid(row=row,column=1,columnspan=2,sticky=W+N+E+S)
    gui['progress'] = p

    row +=1
    gui["trialinfo"]  = StringVar()
    gui["trialinfo"].set("Not running")
    l = Label(f,textvariable=gui['trialinfo'],fg='yellow',bg='black')
    l.grid(row=row,column=0,columnspan=3,sticky=W,pady=10,padx=10)

    row += 1
    gui["photo"]=PhotoImage(file='screenshot_base.gif')
    l = Label(f,image=gui['photo'],borderwidth=0,highlightthickness=0)
    l.grid(row=row,column=0,columnspan=5,sticky=W)
    gui["photolabel"]=l

    # Make some elements available for the future
    gui["loadb"]     =loadb
    gui["runb"]      =runb
    gui['recogb']    =recogb
    gui['mcb']       =mcb
    gui["quitb"]     =quitb
    gui["keep_going"]=False
    gui["loaded"]    =False
    gui["running"]   =False
    gui["logging"]   =False

    master.bind()

    return master







def init_mouse():
    if conf['use_mouse']:
        if not os.path.exists(conf['mouse_device']):
            print("## ERROR: mouse device %s does not exist"%conf['mouse_device'])
            sys.exit(-1)
        mouse = MouseInput(conf['mouse_device'])
        conf['mouse']=mouse




## Initialise everything
conf['screen'],conf['mainfont'] = init_interface(conf)
init_mouse()
pygame.display.flip()


master = init_tk()
update_ui()
master.mainloop()
