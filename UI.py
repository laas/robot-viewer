#! /usr/bin/env python

# Copyright LAAS/CNRS 2009-2012
# Authors Duong Dang

from Tkinter import *
from tkFileDialog   import askopenfilename      

def openFileCallback():
    print "opening :\t",askopenfilename()


def dummyCallback():
    print "called the dummyCallback!"

root = Tk()

# create a menu
menu = Menu(root)
root.config(menu=menu)

filemenu = Menu(menu)
menu.add_cascade(label="File", menu=filemenu)
filemenu.add_command(label="Load Robo", command=openFileCallback)
filemenu.add_command(label="Load Motion", command=dummyCallback)
filemenu.add_separator()
filemenu.add_command(label="Exit", command=dummyCallback)

helpmenu = Menu(menu)
menu.add_cascade(label="Help", menu=helpmenu)
helpmenu.add_command(label="About...", command=dummyCallback)

mainloop()
