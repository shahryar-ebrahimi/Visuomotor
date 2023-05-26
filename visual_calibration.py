import pygame
#from Tkinter import * # use for python2
#import tkMessageBox
#import tkFileDialog
import numpy as np
import random
import time
import robot.interface as robot
import pickle
import datetime

from tkinter import *
from tkinter import messagebox as tkMessageBox
from tkinter import filedialog
import tkinter.simpledialog as tksd
from tkinter import messagebox
import tkinter.ttk as ttk
    
# Controlling the subject screen
SUBJ_SCREENSIZE = (1920,1080)
SUBJ_SCREENPOS = (1600,0) # the offset (allows you to place the subject screen "on" the second monitor)

# number of presented targets for calibration
N_HORIZ_VISUAL_TARGETS,N_VERTIC_VISUAL_TARGETS = 9,4
#N_HORIZ_VISUAL_TARGETS,N_VERTIC_VISUAL_TARGETS = 3,2
TARGET_REPETITIONS = 1 # how often to present each target
N_CAPTURE = 50 # how many data points to capture (and average) for each target
CAPTURE_SLEEP = .005 # how long to sleep between captures
XSPAN = (700,1080)
YSPAN = (150,450)
# The target circle radius
TARGET_RADIUS = 10

# The little control window
CONTROL_WIDTH,CONTROL_HEIGHT= 345,600 # control window dimensions
CONTROL_X,CONTROL_Y = 1000,200 # controls where on the screen the control window appears


def init_pygame():

    pygame.init()

    import os
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % SUBJ_SCREENPOS # controls where the subject screen will appear

    global subjscreen
    subjscreen = pygame.display.set_mode(SUBJ_SCREENSIZE,pygame.NOFRAME)
    subjscreen.fill((0,0,0))
    pygame.display.flip()
    targs = [ (int(x),int(y))
              for x in np.linspace(XSPAN[0],XSPAN[1] , N_HORIZ_VISUAL_TARGETS)
              for y in np.linspace(YSPAN[0],YSPAN[1],N_VERTIC_VISUAL_TARGETS)
    ]
    
    global targets
    targets = []
    for _ in range(TARGET_REPETITIONS):
        random.shuffle(targs)
        targets += targs[:]

    global current_target,captured
    current_target = -1
    next_target()

    captured = []
 

def draw_target(target):
    (x,y) = target

    global subjscreen
    
    subjscreen.fill((0,0,0))
    updaterec = pygame.draw.circle(subjscreen,(255,0,0),(x,y),TARGET_RADIUS)
    pygame.display.flip()

def next_target():

    global targets,current_target
    current_target+= 1

    if current_target>=len(targets):
        gui["sampleb"].configure(state=DISABLED)
        write_tofile()
        #follow_robot()
        return
        
    # Now wait for the subject to go there
    target = targets[current_target]
    print("Presenting target",target)
    draw_target(target)

    #global gui
    #gui["target"].set("target %i/%i"%(current_target+1,len(targets)))

def sample():
    capt = []
    for _ in range(N_CAPTURE):
        x,y = robot.rshm('x'),robot.rshm('y')
        capt.append((x,y))
        time.sleep(CAPTURE_SLEEP)
    
    # Capture the position
    allx,ally=zip(*capt)
    capx,capy=np.mean(allx),np.mean(ally)

    global captured,current_target,targets
    targx,targy=targets[current_target]
    captured.append( (targx,targy,capx,capy) )
    gui["message1"].insert(END,"%.6f"%(capx)+"\n" )
    gui["message2"].insert(END,str(targx)+"\n")
    gui["message3"].insert(END,"%.6f"%(capy)+"\n")
    gui["message4"].insert(END,str(targy)+"\n")
    # Show in the interface what we captured
    #global gui
    #gui["position"].set("x=%.3f y=%.3f"%(capx,capy))
    
    next_target()

    #print('we sample the screen')
    



def linregr(x1,x2,y):
    """ Find a linear regression, i.e. slope and intercept, to predict y based on x."""
    A = np.vstack([x1,x2, np.ones(len(x1))]).T
    slope1,slope2, interc = np.linalg.lstsq(A, y)[0]
    return (slope1,slope2,interc)
    
    

def write_tofile():
    global targets,current_target
    #global gui

    #gui["target"].set("Targets complete")

    global captured
    scr_x,scr_y,rob_x,rob_y = zip(*captured)

    regrs = {}
    for (var,screenvar,robotvarx,robotvary) in [("x",scr_x,rob_x,rob_y),
                                     ("y",scr_y,rob_x,rob_y)]:
        sl1,sl2,interc = linregr( robotvarx,robotvary, screenvar )
        regrs["slope1.%s"%var]  = sl1
        regrs["slope2.%s"%var]  = sl2
        regrs["interc.%s"%var] = interc

    #subjid = gui["subject.id"].get()

    timestamp = datetime.datetime.now().strftime("%Y%d%m_%Hh%Mm%S")
    
    #fname = "calib_%s.pickle27"%(timestamp)
    fname = "visual_calib.pickle27"
    pickle.dump((captured,regrs),open(fname,'wb'))

    print("Robot -> Screen regression: ",regrs)
    

def load_robot():
    robot.load()
    gui["sampleb"].configure(state=NORMAL)
    gui["quitb"].configure(state=NORMAL)
    gui["loadb"].configure(state=DISABLED)
    #update_ui()

def unload_robot():
    robot.unload()
    #update_ui()

def endprogram():
    pygame.quit()
    unload_robot()	
    sys.exit(0)
      

def init_tk():
    global gui
    gui = {}
    
    master = Tk()
    master.geometry('%dx%d+%d+%d' % (CONTROL_WIDTH, CONTROL_HEIGHT, CONTROL_X, CONTROL_Y))
    #master.configure(background='black')

    f = Frame(master,background='WHITE')
    #f = master
    loadb   = Button(f, text="Load robot",                background="green",foreground="black",command=load_robot)
    sampleb = Button(f, text="  Sample  " ,               background="blue" ,foreground="black",command=sample, state=DISABLED)
    quitb   = Button(f, text="   quit   ",                background="red"  ,foreground="black",command=endprogram, state=DISABLED)
    gui["loadb"]   =loadb
    gui["sampleb"] =sampleb
    gui["quitb"]   =quitb
    
    posxl      = Label(f, text="Robot X",             fg="BLACK")
    screenxl   = Label(f, text="Screen X",            fg="BLACK")
    posyl      = Label(f, text="Robot Y",             fg="BLACK")
    screenyl   = Label(f, text="Screen Y",            fg="BLACK")
    sb = Scrollbar(f)
    mess1 = Text(f, width=10, height=38, wrap=WORD, yscrollcommand=sb.set,fg='BLACK',bg='WHITE')
    gui["message1"]    = mess1
    mess2 = Text(f, width=10, height=38, wrap=WORD, yscrollcommand=sb.set,fg='BLACK',bg='WHITE')
    gui["message2"]    = mess2
    mess3 = Text(f, width=10, height=38, wrap=WORD, yscrollcommand=sb.set,fg='BLACK',bg='WHITE')
    gui["message3"]    = mess3
    mess4 = Text(f, width=10, height=38, wrap=WORD, yscrollcommand=sb.set,fg='BLACK',bg='WHITE')
    gui["message4"]    = mess4

    row  = 0
    f.grid         (row=row,padx=10,pady=10)
    row += 1
    loadb.grid     (row=row,column=0,sticky=W,padx=10,pady=10)
    sampleb.grid   (row=row,column=1,sticky=W,pady=10)
    quitb.grid     (row=row,column=2,sticky=W,pady=10)
    row += 1
    posxl.grid         (row=row,column=0,sticky=W, pady=10)
    screenxl.grid      (row=row,column=1,sticky=W, pady=10)
    posyl.grid         (row=row,column=2,sticky=W, pady=10)
    screenyl.grid      (row=row,column=3,sticky=W, pady=10)
    row += 1
    sb.config(command=mess1.yview)
    #sb.config(command=mess2.yview)
    #sb.config(command=mess3.yview)
    #sb.config(command=mess4.yview)
    sb.grid( row=row,column=4,sticky=W, ipady=230)
    mess1.grid      (row=row,column=0,sticky=W)
    mess2.grid      (row=row,column=1,sticky=W)
    mess3.grid      (row=row,column=2,sticky=W)
    mess4.grid      (row=row,column=3,sticky=W)
    # Make some elements available for the future
    #gui["position"].set("x=[] y=[]")
    #gui["target"].set("Target NA/NA")
    #gui["loadb"]   =loadb
    #gui["unloadb"] =unloadb
    #gui["zerob"]   =zerob
    #gui["loadcalb"]=loadcalb
    #gui["quitb"]   =quitb
    #gui["keep_going"]=False
    
    master.bind()

    return master

master = init_tk()
init_pygame()
#unload_robot()
#update_ui()
master.mainloop()
