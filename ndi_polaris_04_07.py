#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 16 14:45:59 2017

@author: mwoletz
"""
import socket
import select
import crcmod
from os import path
from glob import glob
import binascii
from enum import IntFlag, IntEnum
import time
from io import BytesIO
from datetime import datetime
from math import ceil
import struct
import numpy as np
#import pandas as pd
from sys import stdout
import pickle


crc16 = crcmod.mkCrcFun(0x18005, 0xffff, True)

class ReplyOptions(IntFlag):
    TRANSFORMATION_DATA          = 1
    TOOL_AND_MARKER              = 2
    POSITION_SINGLE_STRAY_ACTIVE = 4
    POSITION_TOOL_MARKERS        = 8
    NOTREPORTED_TRANSFOMRATIONS  = 2048
    POSITION_STRAY_PASSIVE       = 4096

class HandleStatus(IntEnum):
    VALID = 1
    MISSING = 2
    DISABLED = 4
    
ProbeTypes = {'01': 'Reference',
              '02': 'Probe',
              '03': 'Button Box',
              '04': 'Software-defined',
              '05': 'Microscope tracker',
              '06': 'Reserved',
              '07': 'Calibration device',
              '08': 'Strober or Tool Docking Station',
              '09': 'Isolation box',
              '0A': 'C-arm tracker',
              '0B': 'Catheter',
              '0C': 'GPIO device'}

ToolInformations = ['Bad transformation fit',
                    'Not enough acceptable markers for transformation',
                    'IR interference',
                    'Fell behind while processing',
                    'Tool face 0 used',
                    'Tool face 1 used',
                    'Tool face 2 used',
                    'Processing exception']
MarkerInformations = {
                    0: 'Not used - Missing',
                    1: 'Not used - Exceeded maximum marker angle',
                    2: 'Not used - Exceeded maximum 3D error for tool',
                    3: 'Used',
                    4: 'Used - but out of volume',
                    5: 'Not used - Outside measurement volume and not needed'
                    }

PortStatus = {
             0 : 'Occupied',
             1 : 'Switch 1 closed/GPIO line 1 active',
             2 : 'Switch 2 closed/GPIO line 2 active',
             3 : 'Switch 3 closed/GPIO line 3 active',
             4 : 'Initialized',
             5 : 'Enabled',
             6 : 'Out of volume',
             7 : 'Partially out of volume',
             8 : 'Algorithm limitation',
             9 : 'IR interference',
             12: 'Processing exception',
             14: 'Fell behind while processing',
             15: 'Data buffer limitation'}

SystemStatus = {
        0: 'System communication synchronization error',
        3: 'Recoverabke system processing exception',
        6: 'Some port handle has become occupied',
        7: 'Some port handle has become unoccupied',
        8: 'Diagnostic pending',
        9: 'Temperature'
        }

binary_ref_rom = b'NDI\x00b\x0f\x00\x00\x01\x00\x00\x00\x03\x00\x00\x01\x00\x00\x00\x00\x00l\xdd<Z\x00\x00\x00\x03\x00\x00\x00\x03\x00\x00\x00\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00 A\x00\x00\x00\x00\x00\x00\x80?\x00\x00\x00@\x00\x00@@\x00\x00\x80@\x00\x00\xa0@\x00\x00\xc0@\x00\x00\xe0@\x00\x00\x00A\x00\x00\x10A\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x1f\x1f\x1f\x1f\t\x00\x00\x00MUW\x00\x00\x00\x00\x00\x00\x00\x00\x00Test1\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\t\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80\x00)\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x80?\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
plate_roms = []

"""
What we know:
    The positional Arguments of the roms are from 72 to 108 (72 + 9*4 Bytes) 
    9 poitional Arguments in float, each 4 bytes
    bytes is not subscriptable so we need to convert it to a list format
    once ist is listed, each float can be asigned to the 4 bytes.
    Rom Name is : 592 til 611
"""

def create_rom_file(name,X): 
    
    coord = struct.pack('<9f',*X)
    name = name[:20]
    name_bytes = name.ljust(20,'\00').encode('ascii')
    rom_data = BytesIO(binary_ref_rom)
    rom_data.seek(592)
    rom_data.write(name_bytes)
    rom_data.seek(72)
    rom_data.write(coord)
    rom_data.seek(0)
    rom = rom_data.read()
    checksum= sum(rom[6:])
    checksum_bytes = struct.pack('i',checksum)
    rom_data = BytesIO(rom)
    rom_data.seek(4)
    rom_data.write(checksum_bytes)
    rom_data.seek(0)
    rom = rom_data.read()
    return rom
    
    

def call_position_plate():
    point_file_path = '/ceph/mri.meduniwien.ac.at/departments/physics/fmrilab/home/dcolin/solution_20211115-151419.pkl'
    with open(point_file_path, 'rb') as fp:
        data = pickle.load(fp)
    
    #X = data['X'] 
    #X -= (X.max(0) + X.min(0)) / 2

    return data

def triangle(data,i):
    tri = data['X'][data['triangles'][i]] #get positional data
    tri = tri - tri.mean(0) # normalize to center of endpoints
    tri = np.c_[tri,np.zeros(3)].flatten() #make 1D array
    return tri 

def get_plate_roms():
    data = call_position_plate()
    for i,triangle in enumerate(data['triangles']):
        X = data['X'][triangle]*10
        X = X - X.mean(0)
        X = np.c_[X,np.zeros(3)].flatten() #Ads a column of 0 (z coords) and turns it into 1d array for create rom
        rom = create_rom_file(f'plate{i}',X)
        plate_roms.append(rom)
        print(triangle,X)

#np.c_[data['X'][data['triangles'][1]],np.zeros(3)].flatten()
   
def int_to_port_status(status):
    return [PortStatus[k] for k in PortStatus.keys() if status & (1 << k)]

def int_to_system_status(status):
    return [SystemStatus[k] for k in SystemStatus.keys() if status & (1 << k)]

def byte_to_bit_bool(n):
    return [bool(n & (1 << i)) for i in range(8)]

def bytes_to_4_bits(bs):
    return [(b >> s) & 15 for b in bs for s in [0,4]]


def bx_get_message(s, options):
    # s is s,__self = a port
    now = datetime.now()
    time_stamp = [now.year, now.month, now.day, now.hour, now.minute, now.second, now.microsecond]
    
    start_sequence = 0xa5c4#42436
    start_b = s.recv(2)
    #start_b = answer[0:2]
    start, = struct.unpack('<H', start_b)
    
    """
    if start != start_sequence:
        time.sleep(0.01)
        #data_left = s.inWaiting()  # Get the number of characters ready to be read
        data_left = s.CMSG_LEN()
        return start_b + s.recv(data_left)
    """
    reply_length, header_crc = struct.unpack('<HH', s.recv(4))
    #reply_length, header_crc = struct.unpack('<HH', answer[2:6])
    
    msg = s.recv(reply_length + 2)
    #msg = answer[6:reply_length+2]
    #save the response in msg
    #everytime you read from msg use "callable" bytes then redefine and cut out sued bytes
    
    orig_msg = msg
    
    dat_handles = []
    
    n_handles = msg[0]
    msg = msg[1:]
    
    for i in range(n_handles):
        dat = {}
        
        if len(msg) >= 2:
            handle, status = struct.unpack('<BB', msg[0:2])
        else:
            print('\n{} of {} handles'.format(i+1, n_handles))
            print('Reply length: {}'.format(reply_length))
            print(dat_handles)
            print(orig_msg)
            
        msg = msg[2:]
        dat['handle'] = '{:02X}'.format(handle)
        status = HandleStatus(status)
        dat['status'] = status
        
        if status != HandleStatus.DISABLED:
            # Reply Option 0001
            if ReplyOptions.TRANSFORMATION_DATA in options:
                if status != HandleStatus.MISSING:
                    q0, qx, qy, qz, tx, ty, tz, error, port_status, frame_number = struct.unpack('<8f2I', msg[0:40])
                    msg = msg[40:]
                    
                    dat['quaternion']   = np.array([q0, qx, qy, qz])
                    dat['translation']  = np.array([tx, ty, tz])
                    dat['error']        = error
                    dat['port_status']  = int_to_port_status(port_status)
                    dat['frame_number'] = frame_number
                else:
                    port_status, frame_number = struct.unpack('<2I', msg[0:8])
                    msg = msg[8:]
                    
                    dat['port_status']  = int_to_port_status(port_status)
                    dat['frame_number'] = frame_number
            
            # Reply Option 0002
            if ReplyOptions.TOOL_AND_MARKER in options:
                binary_tool_information = byte_to_bit_bool(msg[0])
                tool_information = [ToolInformations[t] for t in range(8) if binary_tool_information[t]]
                msg = msg[1:]
                
                dat['tool_information'] = [binary_tool_information, tool_information]
                
                marker_information_packed = struct.unpack('<10B', msg[0:10])
                msg = msg[10:]
                
                marker_information = bytes_to_4_bits(marker_information_packed)                
                
                dat['marker_information'] = [MarkerInformations.get(mi) for mi in marker_information]
                                
            # Reply Option 0008
            if ReplyOptions.POSITION_TOOL_MARKERS in options:
                n_markers = msg[0]
                msg = msg[1:]
                dat['n_markers'] = n_markers
                
                out_of_volume = msg[0:ceil(n_markers/8)]
                msg = msg[ceil(n_markers/8):]
                dat['marker_out_of_volume'] = [b for B in out_of_volume for b in byte_to_bit_bool(B)]
                
                markers = []
                
                for j in range(n_markers):
                    mtx,mty,mtz = struct.unpack('<3f', msg[0:12])
                    msg = msg[12:]
                    markers += [np.array([mtx,mty,mtz])]
                    
                dat['markers'] = markers
                            
        dat_handles += [dat]
    
    # Reply Option 1000
    dat_stray_markers = {}
    if ReplyOptions.POSITION_STRAY_PASSIVE in options:
        n_stray_markers = msg[0]
        msg = msg[1:]
        dat_stray_markers['n'] = n_stray_markers
        
        if n_stray_markers > 0:                                            
            out_of_volume_stray = msg[0:ceil(n_stray_markers/8)]
            msg = msg[ceil(n_stray_markers/8):]
            
            dat_stray_markers['out_of_volume'] = [b for B in out_of_volume_stray for b in byte_to_bit_bool(B)]
            
            stray_markers = []
            
            for j in range(n_stray_markers):
                if len(msg) < 12:
#                                print('Not enough stray marker info ({}) for {} markers!: '.format(len(msg), n_stray_markers))
#                                print(orig_msg)
#                                print(dat)
                    pass
                else:
                    stx,sty,stz = struct.unpack('<3f', msg[0:12])
                    msg = msg[12:]
                    stray_markers += [np.array([frame_number,j,stx, sty, stz])]
            dat_stray_markers['markers'] = stray_markers
    system_status, msg_crc = struct.unpack('<HH', msg[0:4])
    msg = msg[4:]
    
    return {'system_status': int_to_system_status(system_status), 'handles': dat_handles, 'stray_markers': dat_stray_markers, 'time_stamp': time_stamp}
    #return dat_handles 
    
def ToolInformation(msg):
    return {'tool_type': 
                       {'main_type': ProbeTypes[msg[0:2].decode('ascii')],
                        'number_of_switches': msg[2],
                        'number_of_visible_leds': msg[3],
                        'sub_type': msg[6:8].decode('ascii')},
            'id': msg[8:20].decode('ascii').strip(),
            'tool_revision': msg[20:23].decode('ascii'),
            'serial_number': msg[23:31].decode('ascii'),
            'port_status': msg[31:33]}

class Polaris(object):
    
    def __init__(self):
        self.__romdir = '../roms'
        self.__trackers = []
        self.__reply_options = ReplyOptions.TRANSFORMATION_DATA | \
                               ReplyOptions.TOOL_AND_MARKER | \
                               ReplyOptions.POSITION_TOOL_MARKERS | \
                               ReplyOptions.POSITION_STRAY_PASSIVE
        
        #copy as global variable:
        
        
        """
        self.__com_port = com
        self.default_serial_settings()
              
        self.s = serial.Serial(port     = self.__com_port,
                                 baudrate = self.__baudrate,
                                 xonxoff  = self.__flow_control_software,
                                 rtscts   = self.__flow_control_hardware1,
                                 dsrdtr   = self.__flow_control_hardware2,
                                 stopbits = self.__stopbits,
                                 parity   = self.__parity)
        
        """
        #IP der Kamera = '193.175.166.82'
        #PORT der Kamera = 8765
        self.__s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.__s.connect(('193.175.166.82', 8765))
        #self.__s.setblocking(0) # Something Something never stop recv
                
        """   
        def default_serial_settings(self):
        default_serial_settings = {
                'baudrate': 9600,
                'databits': serial.EIGHTBITS,
                'parity':   serial.PARITY_NONE,
                'stopbits': serial.STOPBITS_ONE,
                'xonxoff':  False,
                'rtscts':   False,
                'dsrdtr':   False}
        
        self.__baudrate               = default_serial_settings['baudrate']
        self.__databits               = default_serial_settings['databits']
        self.__parity                 = default_serial_settings['parity']
        self.__stopbits               = default_serial_settings['stopbits']
        self.__flow_control_software  = default_serial_settings['xonxoff']
        self.__flow_control_hardware1 = default_serial_settings['rtscts']
        self.__flow_control_hardware2 = default_serial_settings['dsrdtr']
        
        return default_serial_settings
        """
    
    def send(self, cmd):
        self.__s.send(bytes(cmd, 'ascii'))
    
    
    def readall(self):
        ready = select.select([self.__s],[],[], 5)
        if ready[0]:
            data = self.__s.recv(4096)
            return data    
    
    def read_wait(self):
        time.sleep(1)
        cr = b'\r'
        finished = False
        data = b''
        while not finished:
            d = self.__s.recv(1)
            data += d
            if d == cr:
                finished = True
        return data
#        data = self.__s.read()
#        time.sleep(0.01)              # Sleep (or inWaiting() doesn't give the correct value)
#        data_left = self.__s.inWaiting()  # Get the number of characters ready to be read
#        data += self.__s.read(data_left)
#        return data
    
    def reset(self):
        self.send('RESET 1\r')
        """
        default_serial_settings = self.default_serial_settings() # after hard reset, the response will be send using the default settings
        for dss in default_serial_settings.keys():
            setattr(self.__s, dss, default_serial_settings[dss])
        """    
        time.sleep(0.1)
        print(self.readall())
        
    def setup(self, roms=[]):
        if not roms:
            roms = self.get_roms()
            if not roms:
                print('At least one rom must be selected!')
                return
        
        self.__trackers = []
        
        self.readall()
       
        print('INIT\n')
        self.send('INIT \r')
        #print(self.read_wait())
        print(self.readall())
        
        for r, rom in enumerate(roms):
            # create port handle for each tool
            print('PHRQ\n')
            self.send('PHRQ *********1****\r')
            #handle_response = self.readall()
            handle_response = self.read_wait()
            print(handle_response)
            port_handle = handle_response[0:2].decode('ascii')            
            self.__trackers += [{'handle': port_handle, 'rom': rom}]      
            
            if isinstance(rom, str):
                f =  open(rom, 'br')
                data = f.read()
                f.close()
            elif isinstance(rom, bytes):
                data = rom
            
            data_str = binascii.hexlify(data).decode('ascii')
            for i in range(ceil(len(data_str)/128)):
                c = i*128
                chunk = '{:0<128}'.format(data_str[c:c+128])
                addr  = '{:04X}'.format(64*i)
                
                msg = 'PVWR {}{}{}\r'.format(port_handle, addr, chunk)
                self.send(msg)
                print(self.readall())
                
           
               
                    #print(self.read_wait())
                    #print(self.read_wait())
            #According to API not longer needed
            print('PINIT\n')
            self.send('PINIT {}\r'.format(port_handle))
            print(self.readall())
            #print(self.read_wait())
            
            print('PHINF\n')
            self.send('PHINF {}0001\r'.format(port_handle))
            
            self.__trackers[-1] = {**self.__trackers[-1], **ToolInformation(self.read_wait())}
            print(self.__trackers[-1])
            
            self.send('PHINF {}0004\r'.format(port_handle))
            self.__trackers[-1]['part_number'] = self.read_wait()[0:20].decode('ascii').strip()
            print(self.__trackers[-1]['part_number'])
            
            print('PENA\n')
            self.send('PENA {}D\r'.format(port_handle))
            print(self.readall())
            #print(self.read_wait())
            
        print('SAVE\n')
        self.send('SAVE \r')
        print(self.readall())
        #print(self.read_wait())
    
    def close(self):
        self.reset()
        self.__s.close()
        
    def prepare_tracking(self):
        #remove 40
        self.send('TSTART\r')
        print(self.read_wait())
        self.readall()
        
    def perform_tracking(self, duration = 100., record = lambda *_: True):
        msgs = []
        
        GREEN = '\x1b[1;42m' #color codes in ANSI
        RED   = '\x1b[1;41m'
        BLACK = '\x1b[1;40m'
        END   = '\x1b[0m'
        
        port_handle_ids = {}
        for i, tracker in enumerate(self.trackers):
            port_handle_ids[tracker['handle']] = i
            
        local_trackers = [{'name': tracker['part_number'], 'color': BLACK} for tracker in self.trackers]
        
        def tracker_string(lts):
            ts = ''
            for t, lt in enumerate(lts):
                ts += '\t' if t > 0 else ''
                ts += lt['color'] + '  ' + END + ': ' + lt['name']
            return ts
        
        print('')
        stdout.write(tracker_string(local_trackers))
        
        t_end = float('inf')
        has_t_end = False
        store_value = False
        
        self.readall()
        
        first_run = True
        handles = []
        while True:
            if (not has_t_end) and record():
                t_end = time.time() + duration
                has_t_end = True
                store_value = True
                
                if not first_run:
                    print('')
                    print('Starting acquisition!')
                
            self.send('BX {:04X}\r'.format(self.__reply_options))
            msg = bx_get_message(self.__s, self.__reply_options)
            try:
                for hndl in msg['handles']:
                    local_trackers[port_handle_ids[hndl['handle']]]['color'] = GREEN if hndl['status'] == HandleStatus.VALID else RED
            except Exception as ex:
                print(ex)
                print(msg)
                break
            
            if store_value:
                msgs += [msg]
            
            stdout.write('\r' + tracker_string(local_trackers))
            
            time.sleep(0.01)
            
            if time.time() >= t_end:
                break
            
            first_run = False
            handles += msg['handles']
            
            
        return msgs
    
    
    def stop_tracking(self):
        self.send('TSTOP \r')
        print(self.read_wait())
    
    def track(self, duration = 100):
        self.prepare_tracking()
        msgs = self.perform_tracking(duration)
        self.stop_tracking()
        return msgs
            
    
    def get_roms(self):
        roms = glob(path.join(self.__romdir, '*.rom'))
        rom_names = [path.splitext(path.split(rom)[1])[0] for rom in roms]
        
        picked_roms = []
        
        roms_picked = False
        
        while not roms_picked:
            print('Please pick the trackers to use:\n')
            
            for r, rom in enumerate(rom_names):
                
                print("[{}] {} - [{}]".format(r, rom, "add" if r not in picked_roms else "remove"))
            
            print("[{}] Done.".format(len(rom_names)))
            
            pick = input("#: ")
            pick = pick.strip()
            
            if pick in [str(r) for r in range(len(roms))]:
                changed_rom = int(pick)
                if changed_rom in picked_roms:
                    picked_roms.remove(changed_rom)
                else:
                    picked_roms.append(changed_rom)
                    
            if pick == str(len(rom_names)):
                roms_picked = True
                
        return [roms[r] for r in range(len(roms)) if r in picked_roms] 
            
    @property
    def trackers(self):
        return self.__trackers
    
  
