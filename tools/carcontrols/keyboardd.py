#!/usr/bin/env python

# This process publishes joystick events. Such events can be suscribed by
# mocked car controller scripts.


### this process needs pygame and can't run on the EON ###

# import pygame  # pylint: disable=import-error

import cereal.messaging as messaging
from cereal import log

import sys
import termios
from termios import (BRKINT, CS8, CSIZE, ECHO, ICANON, ICRNL, IEXTEN, INPCK,
                     ISTRIP, IXON, PARENB, VMIN, VTIME)
import select

# Indexes for termios list.
IFLAG = 0
OFLAG = 1
CFLAG = 2
LFLAG = 3
ISPEED = 4
OSPEED = 5
CC = 6

def getch():
  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)
  try:
    # set
    mode = termios.tcgetattr(fd)
    mode[IFLAG] = mode[IFLAG] & ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON)
    #mode[OFLAG] = mode[OFLAG] & ~(OPOST)
    mode[CFLAG] = mode[CFLAG] & ~(CSIZE | PARENB)
    mode[CFLAG] = mode[CFLAG] | CS8
    mode[LFLAG] = mode[LFLAG] & ~(ECHO | ICANON | IEXTEN)
    mode[CC][VMIN] = 1
    mode[CC][VTIME] = 0
    termios.tcsetattr(fd, termios.TCSAFLUSH, mode)

    i, _, _ = select.select( [sys.stdin], [], [], 0.1)
    if (i):
      ch = sys.stdin.read(1)
    else:
      ch = ""
  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  return ch


DesireStatus = log.Joystick.DesireStatus
current_desire = DesireStatus.laneFollow
current_cruise_offset_MPH = 0
keyboard_sock = messaging.pub_sock('testJoystick')
while True:
  # reset commit status
  dat = messaging.new_message('testJoystick')
  dat.testJoystick.commit = False
  ch = getch()
  if ch == 'l':
    current_desire = DesireStatus.leftLaneChange
  elif ch == 'r':
    current_desire = DesireStatus.rightLaneChange
  elif ch == 'f':
    current_desire = DesireStatus.laneFollow
  elif ch == 'c':
    dat.testJoystick.commit = True
  elif ch == '1':
    current_cruise_offset_MPH += 1
  elif ch == '2':
    current_cruise_offset_MPH -= 1
  elif ch == '3':
    current_cruise_offset_MPH = 0
  else:
    continue
  dat.testJoystick.desire = current_desire
  dat.testJoystick.cruiseOffset = current_cruise_offset_MPH
  print(ch, current_cruise_offset_MPH)
  keyboard_sock.send(dat.to_bytes())
  
