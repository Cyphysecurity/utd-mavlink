#!/usr/bin/env python3

import tkinter as tk
import tkinter.messagebox as msgbox
import PIL.ImageTk as itk
import geotiler
from collections import deque
from copy import deepcopy
from threading import Thread
from mavlinksensormitm import SimulatorProxy
import re

class GUI(tk.Frame):

    def __init__(self, master: tk.Tk, **kwargs):
        tk.Frame.__init__(self, master, kwargs)
        self.__master = master
        self.__proxy = SimulatorProxy(port=14560, raddr='127.0.0.1')
        # GPS Canvas and associated data elements
        self.__canvas = tk.Canvas(master=self, bg='#c0c0c0', width=768, height=600)
        self.__canvas.place(in_=self, x=5, y=3)
        self.__master.after(ms=2000, func=self.__draw)
        self.__mapextent = None
        self.__mapimg = None
        self.__buffer = deque(iterable=[], maxlen=4096)
        # Proxy connection settings
        tk.Label(master=self, text='MAVLink proxy settings').place(in_=self, x=780, y=5, anchor=tk.NW)
        self.__raddr_var = tk.StringVar()
        self.__raddr_var.set('127.0.0.1')
        tk.Label(master=self, text='IP Address').place(in_=self, x=840, y=30, anchor=tk.NE)
        tk.Entry(master=self, textvariable=self.__raddr_var, width=15).place(in_=self, x=840, y=30, anchor=tk.NW)
        self.__rport_var = tk.IntVar()
        self.__rport_var.set(14560)
        tk.Label(master=self, text='Port').place(in_=self, x=840, y=55, anchor=tk.NE)
        tk.Entry(master=self, textvariable=self.__rport_var, width=6).place(in_=self, x=840, y=55, anchor=tk.NW)
        tk.Label(master=self, text='UDP').place(in_=self, x=880, y=55, anchor=tk.NW)
        tk.Button(master=self, text='Start', command=self.__startProxy).place(in_=self, x=950, y=50, anchor=tk.W)
        # Tamper GPS checkbox
        self.__tgps_var = tk.BooleanVar()
        self.__tgps_var.set(False)
        tk.Checkbutton(master=self, text='Tamper GPS', variable=self.__tgps_var, command=self.__proxy.toggleTamperGPS).place(in_=self, x=780, y=80, anchor=tk.NW)
        #self.__proxy.start()

    def __startProxy(self):
        raddr = self.__raddr_var.get().strip()
        rport = self.__rport_var.get()
        IPV4_REGEX = r'^(?:(?:25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)[.]){3}(?:25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)$'
        if rport < 1 or rport > 65535:
            msgbox.showerror('Invalid port', 'The chosen port is out of range: 1-65535')
            self.__rport_var.set(14560)
        elif re.match(IPV4_REGEX, raddr) is None:
            msgbox.showerror('Invalid address', 'Invalid IPv4 address: {:s}'.format(raddr))
            self.__raddr_var.set('127.0.0.1')
        #self.__proxy = SimulatorProxy(port=14560, raddr='192.168.10.5')

    def __genmap(self, last: tuple):
        m = geotiler.Map(center=(last[1], last[0]), zoom=18, size=(768, 600))
        self.__mapextent = deepcopy(m.extent)
        img = geotiler.render_map(m)
        self.__mapimg = itk.PhotoImage(img)

    def __inextent(self, coord: tuple) -> bool:
        return (
            coord[0] > self.__mapextent[0] and
            coord[0] < self.__mapextent[2] and
            coord[1] > self.__mapextent[1] and 
            coord[1] < self.__mapextent[3]
        )
    
    def __relcoords(self, lat: float, lon: float):
        xscale = float(768) / (self.__mapextent[2] - self.__mapextent[0])
        x = abs(int((lat - self.__mapextent[0]) * xscale))
        yscale = float(600) / (self.__mapextent[3] - self.__mapextent[1])
        y = abs(int((lon - self.__mapextent[3]) * yscale))
        return x, y

    def __drawgps(self, coords: tuple):
        if self.__inextent((coords[1], coords[0])):
            x, y = self.__relcoords(coords[1], coords[0])
            self.__canvas.create_oval(x-1, y-1, x+1, y+1, fill='red', outline='red')
        if self.__inextent((coords[3], coords[2])):
            x, y = self.__relcoords(coords[3], coords[2])
            self.__canvas.create_oval(x, y, x+2, y+2, fill='blue', outline='blue')

    def __draw(self):
        self.__canvas.delete('all')
        if self.__proxy is not None:
            for i in self.__proxy.getGPSBuffer():
                self.__buffer.append(i)
        if len(self.__buffer) > 0:
            last = deepcopy(self.__buffer[len(self.__buffer) - 1])
            try:
                if self.__mapextent is None or not (self.__inextent((last[1], last[0])) and self.__inextent((last[3], last[2]))):
                    self.__genmap(last)
                self.__canvas.create_image(0, 0, image=self.__mapimg, anchor=tk.NW)
                for i in self.__buffer:
                    self.__drawgps(i)
                self.__canvas.create_rectangle(675,595,763,555,fill='white')
                self.__canvas.create_text(760,574,justify=tk.RIGHT,font='arial 12',text='real',anchor=tk.SE)
                self.__canvas.create_rectangle(680,570,690,560,fill='red')
                self.__canvas.create_text(760,594,justify=tk.RIGHT,font='arial 12',text='tampered',anchor=tk.SE)
                self.__canvas.create_rectangle(680,590,690,580,fill='blue')
            except ValueError:
                self.__mapextent = None
                self.__mapimg = None
        else:
            self.__canvas.create_text(384, 300, justify=tk.CENTER, font='arial 32', text='No GPS data')
        self.__master.after(ms=40, func=self.__draw)


if __name__ == '__main__':
    root = tk.Tk()
    root.wm_resizable(width=False, height=False)
    root.wm_title(string='MAVLink Sensor MitM')
    GUI(root, width=1024, height=610).pack()
    root.mainloop()
