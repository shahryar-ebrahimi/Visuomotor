import pygame
#from Tkinter import * # use for python2
#import tkMessageBox
#import tkFileDialog
import numpy as np
import random
import time
import robot.interface as robot
import pickle
import os
import math
import json
#import tkMessageBox
import struct

from tkinter import *
from tkinter import messagebox as tkMessageBox
from tkinter import filedialog
import tkinter.simpledialog as tksd
from tkinter import messagebox
import tkinter.ttk as ttk


    
# Controlling the subject screen
SUBJ_SCREENSIZE = (1920,1080)
SUBJ_SCREENPOS = (1600,0) # the offset (allows you to place the subject screen "on" the second monitor)

# The target circle radius
TARGET_RADIUS = 15
# Arc charactristics
#ARC_RADIUS = math.pi/4
#ARC_XDIM = 500
#ARC_YDIM = 420
#ARC_WIDTH =10
# Double arc charactristics
Ro=0.036
Ri=0.034
# The subject cursor redius
CURSOR_RADIUS = 5

# The little control window
CONTROL_WIDTH,CONTROL_HEIGHT= 345,200 # control window dimensions
CONTROL_X,CONTROL_Y = 1000,200 # controls where on the screen the control window appears

# Movement time to brnig the subject back to center
MOVE_TIME= 1.5

# Define color for our shapes
BLACK = (0,0,0)
WHITE =(255,255,255)
BLUE=(0,0,255)
GREEN= (0,255,0)
RED=(255,0,0)
YELLOW=(255,255,0)
# current file directory
curpath=os.getcwd()

def init_pygame():

    pygame.init()

    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % SUBJ_SCREENPOS # controls where the subject screen will appear

    global subjscreen
    subjscreen = pygame.display.set_mode(SUBJ_SCREENSIZE,pygame.NOFRAME)
    subjscreen.fill(BLACK)
    pygame.display.flip()
    
def draw_target(target,CC,TC,AC):
    
    (x,y) = target
    global subjscreen, STX,STY
    #updateAC = pygame.draw.arc(subjscreen,AC,[x+(ARC_WIDTH-ARC_XDIM)*math.cos(ARC_RADIUS)/2-ARC_XDIM/2, y+(ARC_YDIM-ARC_WIDTH)*math.sin(ARC_RADIUS)/2-ARC_YDIM/2, ARC_XDIM, ARC_YDIM],-ARC_RADIUS ,ARC_RADIUS , ARC_WIDTH)
    updateAC = draw_doubleArc((x,y),AC)
    updateCC = pygame.draw.circle(subjscreen,CC,(x,y),TARGET_RADIUS)
    updateTC = pygame.draw.circle(subjscreen,TC,(STX,STY),TARGET_RADIUS)
    #updateTC = pygame.draw.circle(subjscreen,TC,(x,int(y+(ARC_YDIM-ARC_WIDTH)*math.sin(ARC_RADIUS))),TARGET_RADIUS)
    return (updateCC,updateTC,updateAC)
    
def draw_doubleArc(center,color):
    (xs,ys)=center
    (xr,yr)= screen_to_robot(xs,ys)
    t=np.linspace(-np.pi/2,np.pi/2,50)
    xo=Ro*np.cos(t)
    yo=Ro*np.sin(t)
    xi=Ri*np.cos(t)
    yi=Ri*np.sin(t)
    trajx=np.concatenate((xo,-xi,-xo,xi),axis=0)+xr
    trajy=np.concatenate((yo+Ro,yi+Ri+2*Ro,2*Ro+Ri-yo,Ro-yi),axis=0)+yr-(Ro-Ri)/2
    #coords = [ robot_to_screen(x,y) for (x,y) in traj[::10] ]
    #trajs =robot_to_screen(trajx,trajy)
    trajs =[robot_to_screen(x,y) for (x,y) in zip(trajx,trajy)]
    pygame.draw.polygon(subjscreen,color,trajs,0)
    
def draw_cursor(sx,sy,CC,TC,AC):
    global subjscreen,oldrect,SCX,SCY
    subjscreen.fill(BLACK)
    updateCC, updateTC, updateAC = draw_target((SCX,SCY),CC,TC,AC)
    # We shouldn't update/flip the screen yet

    updaterec = pygame.draw.circle(subjscreen,YELLOW,(sx,sy),CURSOR_RADIUS)
    pygame.display.update([updaterec,oldrect,updateCC,updateTC,updateAC])
    #pygame.display.flip()
    oldrect = updaterec

def screen_to_robot(x,y):
    """ Given screen coordinates, find the robot coordinates. """
    global calib
    inv_calib = np.linalg.inv([[calib["slope1.x"],calib["slope2.x"]],[calib["slope1.y"],calib["slope2.y"]]])
    return ((x-calib["interc.x"])*inv_calib[0][0] + (y-calib["interc.y"])*inv_calib[0][1],
            (y-calib["interc.y"])*inv_calib[1][0] + (y-calib["interc.y"])*inv_calib[1][1])
    

def robot_to_screen(x,y):
    """ Given robot coordinates, find the screen coordinates. """
    global calib
    return (int(calib["interc.x"] + (x*calib["slope1.x"]) + (y*calib["slope2.x"])),
            int(calib["interc.y"] + (x*calib["slope1.y"]) + (y*calib["slope2.y"])))

def subj_startpos():
    global SCX,SCY,master,CX,CY,TX,TY,calib,STX,STY
    fname =  curpath+"/"+gui["subject.id"].get()+"_center.txt"
    if os.path.isfile(fname):
	#print("file already exist!")
        rx,ry = pickle.load(open(fname,'rb'))
	
    else:
        rx,ry = robot.rshm("x"),robot.rshm("y")
    
    pickle.dump((rx,ry),open(fname,'wb'))
    fname =  curpath+"/"+gui["subject.id"].get()+"_calibCenter.txt"
	#with open(fname,'w') as outfile:
	#     json.dump({"calib":calib,"rx":rx,"ry":ry},outfile)
    outfile=open(fname,'wb')
    outdata=[calib["interc.x"],calib["slope1.x"],calib["slope2.x"]]
    outdata=outdata+[calib["interc.y"],calib["slope1.y"],calib["slope2.y"]]
    outdata=outdata+[rx,ry]
    outfile.write(struct.pack('8d',*outdata))
    outfile.close()
	
    CX,CY = rx,ry       
    SCX,SCY = robot_to_screen(rx,ry)
    #STX,STY =SCX, int(SCY+(ARC_YDIM-ARC_WIDTH)*math.sin(ARC_RADIUS))
    TY=CY+2*(Ro+Ri)
    (STX,STY)= robot_to_screen(CX,TY)
    TX,TY=screen_to_robot(STX,STY)
    #print("our center and target coordinates are:",CX,CY,TX,TY)

def init_draw():
    global master,gui,oldrect,SCX,SCY 
    subj_startpos()
    draw_target((SCX,SCY),WHITE,WHITE,RED)
    pygame.display.flip()

    """ Draw the robot position on the screen, repeatedly."""
    oldrect = None # just so that it is initialised
    
    gui["keep_going"] = True # set gui["keep_going"] to False to drop out of this and stop following the robot handle
    #update_ui()
    while gui["keep_going"]:
        rx,ry = robot.rshm('x'),robot.rshm('y')
        sx,sy = robot_to_screen(rx,ry)
        draw_cursor(sx,sy,WHITE,WHITE,RED)
        
        master.update_idletasks()
        master.update()


def load_robot():
    global outputfname, Ro,Ri
    gui["Runb"].configure(state=NORMAL)
    gui["quitb"].configure(state=NORMAL)
    gui["loadb"].configure(state=DISABLED)
    gui["opend"].configure(state=DISABLED)
    fname = gui["Exp.Design"].get()
    if fname== curpath+"/generalization_design.des":
       # for generalization experiment the length of movement is 20 cm
       Ro = 0.051
       Ri = 0.049
    #print(" File name is:",Ro) 
    outputfname =  curpath+"/Data/"+gui["subject.id"].get()+gui["Block.No"].get()+".dat"
    if os.path.isfile(outputfname):
       if tkMessageBox.askokcancel('Overwrite', 'File already exist overwrite?'):
          robot.load()
          time.sleep(0.2)
          init_draw()
       else:
          gui["Runb"].configure(state=DISABLED)
          gui["loadb"].configure(state=NORMAL)        
    else:
       robot.load()
       time.sleep(0.2)
       init_draw()
    
    #update_ui()

def unload_robot():
    robot.unload()
    #update_ui()

def endprogram():
    gui["keep_going"]=False
    gui["run_going"] = False
    robot.controller(0)
    subjscreen.fill(BLACK)
    pygame.display.flip()
    robot.stop_log()
    pygame.quit()
    unload_robot()	
    sys.exit(0)
 

def Run():
    global outputfname,mydesign,CX,CY,TX,TY,Maxv,CC,TC,SCX,SCY
    # CC specifies center point's color and TC specifies target color
    CC=WHITE
    TC=WHITE
    AC=RED
    gui["Runb"].configure(state=DISABLED)
    gui["opend"].configure(state=DISABLED)
    robot.wshm('fvv_trial_phase', 0)
    gui["keep_going"]=False
    # before we start the first tiral we make sure we move thev subject to the center point.
    goToCenter(MOVE_TIME)
    rx,ry = robot.rshm('x'),robot.rshm('y')
    dx,dy =rx-CX, ry-CY
    dr= math.sqrt(dx**2+dy**2)
    while dr>0.001:
        rx,ry = robot.rshm('x'),robot.rshm('y')
        dx,dy =rx-CX, ry-CY
        dr= math.sqrt(dx**2+dy**2)
        sx,sy = robot_to_screen(rx,ry)
    draw_cursor(sx,sy,CC,TC,AC)
    # now that we made sure subject is in the center (dr<1mm) we are ready to start the experiment
    robot.wshm('fvv_trial_phase', 1)
    robot.wshm('fvv_trial_no',0)
    robot.start_log(outputfname,9)
    robot.stay_fade(CX,CY)
    time.sleep(0.2)
    # read the design file	
    #fname = curpath+"/exp_design.txt"
    fname = gui["Exp.Design"].get()
    read_design_file(fname)
    
    current_trial = -1
    time_elapsed =0
    clock = pygame.time.Clock()
    gui["run_going"] = True
    # This is the main loop of experiment, it always run untill last tiral in the expriment design file
    while gui["run_going"]:
       if not ((robot.rshm('fvv_trial_phase')==2 and CursorStatus== "off") or robot.rshm('fvv_trial_phase')==6):
           if robot.rshm('fvv_trial_phase')==2 and ArcStatus=="off":
              AC=BLACK
           else:
              AC=RED
       rx,ry = robot.rshm('x'),robot.rshm('y')
       sx,sy = robot_to_screen(rx,ry)
       draw_cursor(sx,sy,CC,TC,AC)
       dt = clock.tick()
       time_elapsed +=dt
 	#fvv_trial_phase==1 indicate robot arm is in center
       rx,ry = robot.rshm('x'),robot.rshm('y')
       dx,dy =rx-CX, ry-CY
       dtx,dty=rx-TX,ry-TY
       dr= math.sqrt(dx**2+dy**2)
       dtr = math.sqrt(dtx**2+dty**2)
       vx, vy = robot.rshm('fsoft_xvel'), robot.rshm('fsoft_yvel')
       vtan = math.sqrt(vx**2+vy**2)
	
       if robot.rshm('fvv_trial_phase')==1 and dr<0.05 and vtan <0.01 and time_elapsed>random.randrange(800,1500):  
	       current_trial+=1 ## start the trial, Go to the next trial
	       print("curent trial is:",current_trial)
       if current_trial>=len(mydesign["trials"]):
           gui["run_going"]=False
           robot.controller(0)
           subjscreen.fill(BLACK)
           pygame.display.flip()
           robot.stop_log()   
       else:
           CursorStatus= mydesign["trials"][current_trial]["cursor"]
           ArcStatus= mydesign["trials"][current_trial]["arc"]
           if "feedback" in mydesign["trials"][current_trial]:
                FeedbackStatus = mydesign["trials"][current_trial]["feedback"]
           else:
                FeedbackStatus= "off"
           robot.wshm('fvv_trial_no',current_trial+1)
           CC=GREEN
           time_elapsed = 0
           Maxv=0
           rx,ry = robot.rshm('x'),robot.rshm('y')
           sx,sy = robot_to_screen(rx,ry)
           draw_cursor(sx,sy,CC,TC,AC)
           if FeedbackStatus=="on":
               robot.start_capture()
               robot.wshm('fvv_trial_phase',2) 
	   
    if robot.rshm('fvv_trial_phase') ==2 and vtan>Maxv:
	   # we need Maxv for subject feedback 
       Maxv = vtan
	   #print("dtr is=",dtr)
    if robot.rshm('fvv_trial_phase') ==2 and dtr<0.05 and vtan<0.01:
       #Then subject reach the target
       time_elapsed = 0
       CC=WHITE
       robot.wshm('fvv_trial_phase',3) 
       robot.stay()
       if Maxv < 0.3:
           #subject was too slow
           TC =BLUE
       elif Maxv>0.4:
           # subject was too fast
           TC=RED
       else:
           #subject had correct speed
           TC =GREEN
           robot.wshm('fvv_trial_no',0.0)
    if robot.rshm('fvv_trial_phase') ==3 and time_elapsed>1000.0:
        # now trial is finished we have to move the subject back to center
        goToCenter(MOVE_TIME)
        robot.wshm('fvv_trial_phase',4)
    if robot.rshm('fvv_trial_phase') ==4 and dr<0.005:
       if FeedbackStatus=="on":
            robot.wshm('fvv_trial_phase',5)
            time_elapsed = 0
       else:
            robot.wshm('fvv_trial_phase',1)
            time_elapsed = 0
    if robot.rshm('fvv_trial_phase') ==5:
           traj=list(robot.retrieve_trajectory()) # retrieve the captured trajectory from the robot memory
           coords = [ robot_to_screen(x,y) for (x,y) in traj[::10] ]
           draw_trajectory(coords)
           robot.wshm('fvv_trial_phase',6)
    if robot.rshm('fvv_trial_phase') ==6 and time_elapsed>2000.0:
           updateTRAJ=pygame.draw.lines(subjscreen,BLACK,0,coords,2)
           updateAC = draw_doubleArc((SCX,SCY),AC)
           pygame.display.update([updateTRAJ,updateAC])
           robot.wshm('fvv_trial_phase',1)
    master.update_idletasks()
    master.update()

def draw_trajectory(coords):
    """ Draw the trajectory that we have captured. """
    global subjscreen,SCX,SCY, STX,STY
    #print("I'll draw the trajectory later....")
    CC=WHITE
    TC=WHITE
    AC=RED
    #updateAC = pygame.draw.arc(subjscreen,AC,[SCX+(ARC_WIDTH-ARC_XDIM)*math.cos(ARC_RADIUS)/2-ARC_XDIM/2, SCY+(ARC_YDIM-ARC_WIDTH)*math.sin(ARC_RADIUS)/2-ARC_YDIM/2, ARC_XDIM, ARC_YDIM],-ARC_RADIUS ,ARC_RADIUS , ARC_WIDTH)
    updateAC = draw_doubleArc((SCX,SCY),AC)
    updateCC = pygame.draw.circle(subjscreen,CC,(SCX,SCY),TARGET_RADIUS)
    #updateTC = pygame.draw.circle(subjscreen,TC,(SCX,int(SCY+(ARC_YDIM-ARC_WIDTH)*math.sin(ARC_RADIUS))),TARGET_RADIUS)
    updateTC = pygame.draw.circle(subjscreen,TC,(STX,STY),TARGET_RADIUS)
    updateTRAJ=pygame.draw.lines(subjscreen,YELLOW,0,coords,2)
    pygame.display.update([updateCC,updateTC,updateAC,updateTRAJ])
   
    

def read_design_file(mpath):
    global mydesign
    print(mpath)
    if os.path.exists(mpath):
        with open(mpath,'r') as f:
            mydesign = json.load(f)
        print("Design file loaded! Parsing the data...")
        return 1
    else:
        print(mpath)
        print("##Error## Experiment design file not found. Have you created one?")
        return 0

 	
def goToCenter(duration):
    global CX,CY
    """ Move the robot to the center or start position and stay there"""
    #robot.wshm('fvv_trial_phase', 0)

    # Ensure this is a null field first
    robot.controller(0)
    print("  Now moving to center: %f,%f"%(CX,CY))
    # Send command to move to cx,cy
    robot.move_to(CX,CY,duration)
    
    # Put flag to 1, indicating robot handle @ center position
    #robot.wshm('fvv_trial_phase', 1)
    	
   
def load_calib():

    #fname = tkFileDialog.askopenfilename(filetypes=[('pickles','.pickle27')])
    fname =  curpath+'/visual_calib.pickle27'
    if fname!=None:
        print("Opening",fname)

        (captured,regrs) = pickle.load(open(fname,'rb'))
        global calib
        calib = regrs
	#print("calib data:",calib)
        return

def open_design():
    #print(" I'll do nothing")
    fname = tkFileDialog.askopenfilename(filetypes=[('json','.des')])
    gui["Exp.Design"].set(fname)

def init_tk():
    global gui
    gui = {}
    
    master = Tk()
    master.geometry('%dx%d+%d+%d' % (CONTROL_WIDTH, CONTROL_HEIGHT, CONTROL_X, CONTROL_Y))

    f = Frame(master,background='WHITE')
    loadb   = Button(f, text="Load robot",       background="green",foreground="black",command=load_robot)
    Runb = Button(f, text="   Run    "      ,    background="blue" ,foreground="black",command=Run, state=DISABLED)
    quitb   = Button(f, text="   quit   ",       background="red"  ,foreground="black",command=endprogram, state=DISABLED)
    gui["loadb"]   =loadb
    gui["Runb"] =Runb
    gui["quitb"]   =quitb
    gui["subject.id"] = StringVar()
    gui["Block.No"] = StringVar()
    gui["Exp.Design"] = StringVar()
    subjl  = Label(f, text="subject ID",             fg="black", bg="white")
    subjid = Entry(f, textvariable=gui["subject.id"],fg="black", bg="white", width=12)
    gui["subject.id"].set("test")
    blockl  = Label(f, text="Block No.",             fg="black", bg="white")
    blockno = Entry(f, textvariable=gui["Block.No"],fg="black", bg="white", width=5)
    gui["Block.No"].set("1")
    open_designb   = Button(f, text="Open",       background="green",foreground="black",command=open_design)
    gui["opend"]   =open_designb
    openl  = Label(f, text="Design file",             fg="black", bg="white")
    OpenEntry = Entry(f, textvariable=gui["Exp.Design"],fg="black", bg="white", width=12)
    gui["Exp.Design"].set(curpath+"/exp_design.des")
     

    row  = 0
    f.grid            (row=row,padx=10,pady=10)
    row += 1
    loadb.grid        (row=row,column=0,sticky=W,padx=10,pady=10)
    Runb.grid         (row=row,column=1,sticky=W,padx=10,pady=10)
    quitb.grid        (row=row,column=2,sticky=W,padx=10,pady=10)
    row += 1
    subjl.grid        (row=row,column=0,sticky=W,padx=10,pady=(20,5))
    subjid.grid       (row=row,column=1,sticky=W,padx=10,pady=(20,5))
    row += 1
    blockl.grid       (row=row,column=0,sticky=W,padx=10)
    blockno.grid      (row=row,column=1,sticky=W,padx=10)
    row += 1
   
    openl.grid        (row=row,column=0,sticky=W,padx=10)
    OpenEntry.grid    (row=row,column=1,sticky=W,padx=10)
    open_designb.grid (row=row,column=2,sticky=W,padx=10)
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
load_calib()
init_pygame()

#unload_robot()
#update_ui()
master.mainloop()

