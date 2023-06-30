#!/usr/bin/python3

# In putty you need to send Ctrl-J to create \n
            
# IMPORTS
################################################################
import asyncio
import logging
import struct
import argparse
import signal
import math
import time
import os
import uvloop
from copy import copy
import serial
import serial_asyncio
import re

from bleak      import BleakClient, BleakScanner
from bleak.exc  import BleakError
from bleak.backends.characteristic import BleakGATTCharacteristic

asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

################################################################
# Support Functions
################################################################

count = 1


def float_to_hex(f):
    '''Pack float into 8 characters representing '''
    bytes_ = struct.pack('!f', f)                            # Single precision, big-endian byte order, 4 bytes
    hex_strings = [format(byte, '02X') for byte in bytes_]   # Convert each byte to a hexadecimal string
    return ''.join(hex_strings)

def hex_to_float(hex_chars):
    '''Unpack 8 bytes to float'''
    hex_bytes = bytes.fromhex(hex_chars)  # Convert hex characters to bytes
    return struct.unpack('!f', hex_bytes)[0]     

async def update_serial(reader, writer, logger, finish_up):
    '''
    gearVRC test reading over serial
    '''

    while not finish_up.is_set():

        writer.write("v\r\n".encode())
        msg_in = await reader.readline() # read a full line
        line_in = msg_in.decode().strip()

        print(line_in)
                                            
        writer.write(("b{}\r\n".format(count)).encode())
        readings = [0.0 for i in range(9)]
        for j in range(count):
            for i in range(9):                           # 9 values, acc,gyr,mag 3 each for x,y,z
                byte_array = await reader.read(8)        # byte array of 8 bytes for each floating point value (float is 4 bytes and when converted to readable hex is 8 bytes)
                hex_chars = ''.join(byte_array.decode()) # convert byte array to string
                readings[i] = hex_to_float(hex_chars)     #
            await reader.read(2) # consumes remaining '\r\n'
            print(readings)

    logger.log(logging.INFO, 'Serial stopped')
    
async def handle_termination(tasks, logger, finish_up):
    logger.log(logging.INFO, 'Control-C or kill signal detected')
    finish_up.set()
    logger.log(logging.INFO, 'Cancelling all tasks...')
    if tasks is not None:
        await asyncio.sleep(1)
        for task in tasks:
            task.cancel()

async def main(args: argparse.Namespace):

    finish_up  = asyncio.Event()
    
    # Setup logging
    logger = logging.getLogger(__name__)
    logger.log(logging.INFO, 'Starting...')

    logger.log(logging.INFO, 'Creating serial reader and writer with {} at {} baud...'.format(args.serial, args.baud))
    serialReader, serialWriter = await serial_asyncio.open_serial_connection(url=args.serial, baudrate=args.baud)

    tasks = [asyncio.create_task(update_serial(serialReader, serialWriter, logger, finish_up))]   # update serial, will not terminate
 
    # Set up a Control-C handler to gracefully stop the program
    # This mechanism is only available in Unix
    if os.name == 'posix':
        # Get the main event loop
        loop = asyncio.get_running_loop()
        loop.add_signal_handler(signal.SIGINT,  lambda: asyncio.create_task(handle_termination(tasks,logger, finish_up)) ) # control-c
        loop.add_signal_handler(signal.SIGTERM, lambda: asyncio.create_task(handle_termination(tasks,logger, finish_up)) ) # kill

    # These tasks will not terminate 
    await asyncio.wait(tasks, timeout=float('inf'))

    logger.log(logging.INFO,'Exit')

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()

    device_group = parser.add_mutually_exclusive_group(required=False)

    parser.add_argument(
        '-s',
        '--serial',
        dest = 'serial',
        type = str,
        metavar='<serial>',
        help='enables serial output',
        default = '/tmp/ttyV1'
    )

    parser.add_argument(
        '-b',
        '--baud',
        dest = 'baud',
        type = int,
        metavar='<baud>',
        help='serial baud rate',
        default = 115200
    )
    
    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='sets the log level to debug',
        default = False
    )

    args = parser.parse_args()
        
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)-15s %(name)-8s %(levelname)s: %(message)s',
    )   
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass
