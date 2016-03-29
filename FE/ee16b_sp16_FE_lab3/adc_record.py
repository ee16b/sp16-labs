#!/usr/bin/env python

import serial
import sys
import os
import re

samples = []
fs = 333

# Serial functions
def serial_ports():
    """Lists serial ports

    Raises:
    EnvironmentError:
        On unsupported or unknown platforms
    Returns:
        A list of available serial ports
    """
    if sys.platform.startswith('win'):
        ports = ['COM' + str(i + 1) for i in range(256)]

    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this is to exclude your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')

    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')

    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def open_port(port, baud):
    """Open a serial port.

    Args:
    port (string): port to open, on Unix typically begin with '/dev/tty', on
        or 'COM#' on Windows.
    baud (int, optional): baud rate for serial communication

    Raises:
    SerialException: raised for errors from serial communication

    Returns:
       A pySerial object to communicate with the serial port.
    """
    ser = serial.Serial()
    try:
        ser = serial.Serial(port, baud, timeout=10)
        logger.info("Opened serial connection on port %s"%port)
        return ser
    except serial.SerialException:
        raise

def save_txt(buff, txtfile):
  print("Writing to " + txtfile + "...")
  if os.path.isfile(txtfile):
    os.remove(txtfile)
  text_file = open(txtfile, 'w')
  text_file.write('\n'.join([str(x) for x in buff]))
  text_file.close()

def run(filename, debug=0):
  samples = []
  print("EE16B Front End Lab")

  ports = serial_ports()
  if ports:
    print("Available serial ports:")
    for (i,p) in enumerate(ports):
      print("%d) %s"%(i+1,p))
  else:
    print("No ports available. Check serial connection and try again.")
    print("Exiting...")
    return

  portNo = input("Select the port to use: ")
  print('Starting serial communication...')
  ser = serial.Serial(ports[int(portNo)-1])
  ser.baudrate=9600
  ser.timeout=20
  #ser.readline()
  
  print('Catching Start flag... (be sure to press RST)')

  while (not re.match("Start", ser.readline().decode())):
    pass

  print('ADC detected. Start acqusition')

  for i in range(fs*10):
    samples.append((int)(ser.readline().decode().rstrip('\n')))
  save_txt(samples, filename)
  print('Done writing to ' + filename)

  ser.close()
