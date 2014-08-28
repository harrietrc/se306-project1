#!/usr/bin/env python
# Using Tkinter to create a GUI
import tkinter	
#import rospy
#import roslib
# from std_msgs.msg import String
#from se306_project1 import ResidentMsg

# We inherit from Tkinter.Tk, which is the base class for standard windows.
class simpleapp_tk(tkinter.Tk):
    # Constructor
    def __init__(self,parent):
        tkinter.Tk.__init__(self,parent)
        # Keep a reference to our parent.
        self.parent = parent
        self.initialize()
	
    def initialize(self):
        # Create grid layout manager
        self.grid()
        # Create a text box
        self.entry = tkinter.Entry(self)
        self.entry.grid(column=0,row=0,sticky='EW')
        # Create a button
        button = tkinter.Button(self,text="Click me !")
        button.grid(column=1,row=0)
        # Create a label
        label = tkinter.Label(self,anchor="w",fg="white",bg="blue")
        label.grid(column=0,row=1,columnspan=2,sticky='EW')
	
#def listener():
	#rospy.init_node('listener', anonymous=True)
	#rospy.Subscriber("residentStatus", ResidentMsg, residentStatusCallback)
	#rospy.spin()
	
#def residentStatusCallback():
	#rospy.loginfo(rospy.get_caller_id()+"I heard %s",msg.Health)

if __name__ == "__main__":
    # Instantiate class
    app = simpleapp_tk(None)
    # Give a title to window
    app.title('my application')
    # Loop to make GUI wait for user actions
    app.mainloop()
    #listener()
