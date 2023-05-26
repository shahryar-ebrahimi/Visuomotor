


# This is a module that is intended to interface with the input device.
# Originally, this was the interface with the trackball, but then we started
# using it as a mouse input.


import struct

import os
#import fcntl
import io
from fcntl import fcntl, F_GETFL, F_SETFL







"""

Ok, there are some tricky things here. The challenge was to read from the device 
(e.g. /dev/input/mouse1) and get any events that might be waiting. However, if there is nothing waiting,
I wanted the function to return, so that my program could continue (potentially we want to do
something before the user does anything).
It turns out that this is tricky business. It requires what people call "non-blocking mode" reading.
What that means is that you can try reading, and when there is nothing to read, the function returns
immediately and throws an IOerror, which you can catch.

The solution (based on http://eyalarubas.com/python-subproc-nonblock.html) is as follows:
(1) You open the device node for binary reading as usual.
(2) Then, you use fnctl (specific to Linux) to tell the OS that you want non-blocking reading.
(3) Now you can simply read (using the os.read function) and catch an error when there is nothing to read.


"""





class MouseInput:



    device_node = None


    def __init__(self,device_name):

        # Remark: it seems to me that this /dev/input/mice input is deprecated.
        # But anyway, for now this works fine.
        # see http://stackoverflow.com/questions/4855823/get-mouse-deltas-using-python-in-linux
        self.device_node = open( device_name, "rb" );

        ## Now we tell the OS that we want non-blocking mode
        ## http://eyalarubas.com/python-subproc-nonblock.html
        flags = fcntl(self.device_node, F_GETFL) # get current p.stdout flags
        fcntl(self.device_node, F_SETFL, flags | os.O_NONBLOCK) # update to non-blocking mode




    def getEvent(self):

        # See if there is an event, and if so, return the dx and dy.
        try:
            rawread = os.read(self.device_node.fileno(),3)
            #print rawread
            #(buf,) = rawread
            buf = rawread
        except OSError:
            # the os throws an exception if there is no data
            return None # meaning there was no new data
        except ValueError:
            print("len(rawread)",len(rawread))
            print("Don't know what to do with:",rawread)
            return None

        # Still here? Okay, that means there was some data read.
        if len(buf)>0:
            #print(buf)
            button = buf[0] #ord( buf[0] );
            bLeft = button & 0x1;
            bMiddle = ( button & 0x4 ) > 0;
            bRight = ( button & 0x2 ) > 0;
            dx,dy = struct.unpack( "bb", buf[1:] );
            # Notice that these are coded as *change* in position.

            #print ("L:%d, M: %d, R: %d, x: %d, y: %d\n" % (bLeft,bMiddle,bRight, x, y) );
            return (dx,dy,bLeft,bMiddle,bRight)
        else:
            return None # means no event was currently happening




    def purgeEvents(self):
        while self.getEvent()!=None:
            pass
        # Keep going until nothing is happening



    def close(self):
        self.device_node.close()


