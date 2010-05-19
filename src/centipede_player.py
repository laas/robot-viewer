from Tkinter import *
import sys
from client import *

class Application(Frame):

    def __init__(self, master=None):
        self.lines = open(sys.argv[1]).readlines()
        self.num_lines = len(self.lines);
        Frame.__init__(self, master)
        self.sc_val = 0
        self.pack()
        self.createWidgets()

    def updateRobot(self,val):
        no_line = int (val)
        curline = self.lines[no_line]
        words = curline.split()
        extra_dof = len(words)-46
        print extra_dof
        config = [ float(words[i+extra_dof]) for i in range(46)]
        print config
        execute('updateElementConfig','hrp',config)

    def createWidgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "QUIT"
        self.QUIT["fg"]   = "red"
        self.QUIT["command"] =  self.quit

        self.QUIT.pack({"side": "left"})

        self.sc = Scale (self, variable = self.sc_val, \
                             command = self.updateRobot, \
                             orient  = HORIZONTAL, length = 200, \
                             to = self.num_lines -1)
        self.sc.pack({"side": "bottom"})


root = Tk()
app = Application(master=root)
app.mainloop()
root.destroy()
