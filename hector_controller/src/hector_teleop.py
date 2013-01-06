#!/usr/bin/env python

import roslib; roslib.load_manifest('hector_controller')
from Tkinter import *
import rospy
import math

class LogScale(Frame):
    def __init__(self, parent=None):
        Frame.__init__(self, parent)
        self.number = 0
        self.slide = Scale(self, orient=HORIZONTAL, command=self.setValue,
                           length=200, sliderlength=20,
                           showvalue=0, tickinterval=1,
                           fro=4, to=9, font=('Arial',9))
        self.text = Label(self, font=('Arial',18))
        self.slide.pack(side=RIGHT, expand=1, fill=X)
        self.text.pack(side=TOP, fill=BOTH)
        self.unimap = {'4':u'\u2074','5':u'\u2075','6':u'\u2076',
                       '7':u'\u2077','8':u'\u2078','9':u'\u2079'}

    def setValue(self, val):
        self.number = (10**(int(val)))
        self.text.configure(text='10%s' %(self.unimap[val]))

class Gui():

    def callbackTp(self, event):
        gains = rospy.get_param('gains')
        tp, ti, td = gains['p'], gains['i'], gains['d']
        rospy.set_param('gains', {'p': event.widget.get(), 'i': ti, 'd': td})

    def callbackTi(self, event):
        gains = rospy.get_param('gains')
        tp, ti, td = gains['p'], gains['i'], gains['d']
        rospy.set_param('gains', {'p': tp, 'i': event.widget.get(), 'd': td})

    def callbackTd(self, event):
        gains = rospy.get_param('gains')
        tp, ti, td = gains['p'], gains['i'], gains['d']
        rospy.set_param('gains', {'p': tp, 'i': ti, 'd': event.widget.get()})

    def callbackX(self, event):
        [_,y,z] = rospy.get_param('goal')
        x = float(event.widget.get())
        self.goal = [x,y,z]
        rospy.set_param('goal', [x,y,z])

    def callbackY(self, event):
        [x,_,z] = rospy.get_param('goal')
        y = float(event.widget.get())
        self.goal = [x,y,z]
        rospy.set_param('goal', [x,y,z])

    def callbackZ(self, event):
        [x,y,_] = rospy.get_param('goal')
        z = float(event.widget.get())
        self.goal = [x,y,z]
        rospy.set_param('goal', [x,y,z])

    def callbackDelay(self, event):
        rospy.set_param('delay', float(event.widget.get()))

    def distance(self, pose, goal):
        # 3 dimensional distance = sqrt( (x-x2)^2 + (y-y2)^2 + (z-z2)^2 )
        return math.sqrt(sum(map(lambda a,b:(a-b)**2.0, pose, self.goal)))

    def updateWaypointListBox(self):
        waypointLb = self.root.nametowidget("wpFrame.waypointLb")
        #clear listbox
        waypointLb.delete(0,END)
        #add waypoints to listbox
        for waypoint in self.waypoints:
            waypointLb.insert(END, waypoint)

    def updateWaypoint(self):
#        odomData = rospy.wait_for_message('/ground_truth/state', nav_msgs.msg.Odometry, 5.0)
        pose = rospy.get_param('pose')
        
        # 3 dimensional distance = sqrt( (x-x2)^2 + (y-y2)^2 + (z-z2)^2 )
        distance = self.distance(pose, self.goal)
        
        if (distance <= self.threshold and len(self.waypoints) > 0):
            self.goal, self.waypoints = self.waypoints[0], self.waypoints[1:]
            rospy.set_param('goal', self.goal)
            self.updateWaypointListBox()
            self.xVar.set(self.goal[0])
            self.yVar.set(self.goal[1])
            self.zVar.set(self.goal[2])
            self.dVar.set(distance)

        self.root.after(1000, self.updateWaypoint)

    def updateDistance(self):
        pose = rospy.get_param('pose')
        distance = self.distance(pose, self.goal)
        self.dVar.set(distance)
        self.root.after(1000, self.updateDistance)

    def mainloop(self):
        self.root.mainloop()
        
    def __init__(self):

        self.root = Tk()

#        self.waypoints = [[2.0,0.0,2.0],[2.0,2.0,2.0],[0.0,2.0,2.0],[0.0,0.0,2.0]]

        # willow indoor
        self.waypoints = [[-5.0,-1.5,2.0],[-5.0,3.0,2.0],[2.0,5.0,2.0]]

        # canyon outdoor
#        self.waypoints = [[-10.0,35.0,8.0],[15.0,35.0,8.0],[2.0,5.0,8.0]]


        self.threshold = 0.5
        self.odometry_topic = '/ground_truth/state'

        self.gains = rospy.get_param('gains')
        self.tau_p, self.tau_i, self.tau_d = self.gains['p'], self.gains['i'], self.gains['d']
        self.goal = rospy.get_param('goal')
        self.delay = rospy.get_param('delay')

        gainFrame = LabelFrame(self.root, text="Gains", name="gainFrame", padx=15, pady=15)
        gainFrame.grid(row=0, column=0, columnspan=3, rowspan=21)

        Label(gainFrame, text=u'\u03C4\u209A').grid(row=0, column=0)
        tpVar = DoubleVar(value=self.tau_p)
        tpScale = Scale(gainFrame, variable=tpVar, from_=-5, to=10.0, length=500, resolution=0.05)
        tpScale.bind("<ButtonRelease-1>", self.callbackTp)
        tpScale.grid(rowspan=20, row=1, column=0)

        Label(gainFrame, text=u'\u03C4\u2097').grid(row=0, column=1)
        tiVar = DoubleVar(value=self.tau_i)
        tiScale = Scale(gainFrame, variable=tiVar, from_=-1, to=2.0, length=500, resolution=0.005)
        tiScale.bind("<ButtonRelease-1>", self.callbackTi)
        tiScale.grid(rowspan=20, row=1, column=1)

        Label(gainFrame, text=u'\u03C4d').grid(row=0, column=2)
        tdVar = DoubleVar(value=self.tau_d)
        tdScale = Scale(gainFrame, variable=tdVar, from_=-5, to=10.0, length=500, resolution=0.05)
        tdScale.bind("<ButtonRelease-1>", self.callbackTd)
        tdScale.grid(rowspan=20, row=1, column=2)

        goalFrame = LabelFrame(self.root, text="Goal", name="goalFrame", padx=15, pady=15)
        goalFrame.grid(row=1, column=4, columnspan=2)

        Label(goalFrame, text="x").grid(row=1, column=4)
        self.xVar = StringVar(value=self.goal[0])
        xW = Entry(goalFrame, textvariable=self.xVar, width=5)
        xW.bind("<Return>", self.callbackX)
        xW.grid(row=1, column=5)

        Label(goalFrame, text="y").grid(row=2, column=4)
        self.yVar = StringVar(value=self.goal[1])
        yW = Entry(goalFrame, textvariable=self.yVar, width=5)
        yW.bind("<Return>", self.callbackY)
        yW.grid(row=2, column=5)

        Label(goalFrame, text="z").grid(row=3, column=4)
        self.zVar = StringVar(value=self.goal[2])
        zW = Entry(goalFrame, textvariable=self.zVar, width=5)
        zW.bind("<Return>", self.callbackZ)
        zW.grid(row=3, column=5)

        Label(goalFrame, text="distance").grid(row=4, column=4)
        self.dVar = StringVar()
        dW = Entry(goalFrame, textvariable=self.dVar, width=5)
        dW.grid(row=4, column=5)

        Label(self.root, text="delay").grid(row=6, column=4)
        delayVar = StringVar(value=self.delay)
        dW = Entry(self.root, textvariable=delayVar, width=5)
        dW.bind("<Return>", self.callbackDelay)
        dW.grid(row=6, column=5)

        wpFrame = LabelFrame(self.root, text="Waypoints", name="wpFrame", padx=5, pady=5)
        wpFrame.grid(row=9, column=4, columnspan=2)
        
        b = Button(wpFrame, text="start waypoints", command=self.updateWaypoint)
        b.grid(row=9, column=4, columnspan=2)

        waypointLb = Listbox(wpFrame, name="waypointLb")
        waypointLb.grid(row=10, column=5, columnspan=1, sticky=E)
        self.updateWaypointListBox()
        self.updateDistance()


if __name__ == '__main__':
    try:
        gui = Gui()
        gui.mainloop()        
    except rospy.ROSInterruptException: pass

