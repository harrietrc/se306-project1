#!/usr/bin/env python
# Using Tkinter to create a GUI

import os
from multiprocessing import Process, Pipe
import roslib
roslib.load_manifest('se306_project1')
import rospy
from std_msgs.msg import String
#from se306_project1 import ResidentMsg
import Tkinter	

callbackData = 'default'
STDIN = 0
STDOUT = 1
STDERR = 2

# We inherit from Tkinter.Tk, which is the base class for standard windows.
class simpleapp_tk(Tkinter.Tk):
    # Constructor
	def __init__(self,parent):
		Tkinter.Tk.__init__(self,parent)
		# Keep a reference to our parent.
		self.parent = parent
		print('boji listener')
		rospy.init_node('listener', anonymous=True)
		rospy.Subscriber("python/gui", String, residentStatusCallback)
		self.initialize()
	
	def initialize(self):
		# Create grid layout manager
		self.grid()
		# Create a text box
		self.entry = Tkinter.Entry(self)
		self.entry.grid(column=0,row=0,sticky='EW')
		self.entry.insert(0, callbackData)
		# Create a button
		button = Tkinter.Button(self,text="Click me !")
		button.grid(column=1,row=0)
		# Create a label
		label = Tkinter.Label(self,anchor="w",fg="white",bg="blue")
		label.grid(column=0,row=1,columnspan=2,sticky='EW')
		rospy.spin()
'''
def listener():
	print('boji listener')
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("python/gui", String, residentStatusCallback)
	rospy.spin()
'''	
def residentStatusCallback(GUImsg):
	global callbackData
	callbackData = GUImsg.data
	print('in boji callback')
	rospy.loginfo(rospy.get_caller_id()+"I heard %s", GUImsg.data)



if __name__ == "__main__":
    # Instantiate class
	app = simpleapp_tk(None)
    # Give a title to window
	app.title('my application')
    # Loop to make GUI wait for user actions


	app.mainloop()
