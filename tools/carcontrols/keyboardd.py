#!/usr/bin/env python

# This process publishes joystick events. Such events can be suscribed by
# mocked car controller scripts.


### this process needs pygame and can't run on the EON ###

# import pygame  # pylint: disable=import-error
import cereal.messaging as messaging
from cereal import log
from pynput import keyboard

DesireStatus = log.Joystick.DesireStatus
current_desire = DesireStatus.laneFollow
class MyException(Exception):
  pass

with keyboard.Events() as events:
  # Block at most one second
  keyboard_sock = messaging.pub_sock('testJoystick')
  while True:
    # reset commit status
    dat = messaging.new_message('testJoystick')
    dat.testJoystick.commit = False
    event = events.get(0.1)
    if event:
      key = event.key
      try:
        # print('alphanumeric key {0} pressed'.format(key.char))
        if key.char == 'l':
          current_desire = DesireStatus.leftLaneChange
        elif key.char == 'r':
          current_desire = DesireStatus.rightLaneChange
        elif key.char == 'f':
          current_desire = DesireStatus.laneFollow
        elif key.char == 'c':
          dat.testJoystick.commit = True
        elif key == keyboard.Key.esc:
          raise MyException(key)
      except AttributeError:
        pass
    dat.testJoystick.desire = current_desire
    print(dat)
    keyboard_sock.send(dat.to_bytes())