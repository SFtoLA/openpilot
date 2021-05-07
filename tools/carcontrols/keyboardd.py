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

# def on_press(key):
#   try:
#     print('alphanumeric key {0} pressed'.format(key.char))
#     if key.char == 'l':
#       print('left turn')
#     if key.char == 'q' or key == keyboard.Key.esc:
#       raise MyException(key)
      
#   except AttributeError:
#     print('special key {0} pressed'.format(key))

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
      
#   except AttributeError:
#     print('special key {0} pressed'.format(key))
      # if key_pressed.char == 'l':
      #   print('left turn turn turn')
    # print()
    # if event is None:
    #     print('You did not press a key within one second')
    # else:
    #     print('Received event {}'.format(event))

# Collect events until released
# with keyboard.Listener(
#         on_press=on_press) as listener:
#     print("listening")
#     try:
#         listener.join()
#     except MyException as e:
#         print('{0} was pressed'.format(e.args[0]))

# def keyboard_thread():
#   keyboard_sock = messaging.pub_sock('testJoystick')

#   # pygame.init()

#   # # Used to manage how fast the screen updates
#   # clock = pygame.time.Clock()

#   # # Initialize the joysticks
#   # pygame.joystick.init()

#   # # Get count of joysticks
#   # joystick_count = pygame.joystick.get_count()
#   # if joystick_count > 1:
#   #   raise ValueError("More than one joystick attached")
#   # elif joystick_count < 1:
#   #   raise ValueError("No joystick found")

#   # -------- Main Program Loop -----------
#   while True:
#     # EVENT PROCESSING STEP
#     # for event in pygame.event.get():  # User did something
#     #   if event.type == pygame.QUIT:  # If user clicked close
#     #     pass
#     #   # Available joystick events: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
#     #   if event.type == pygame.JOYBUTTONDOWN:
#     #     print("Joystick button pressed.")
#     #   if event.type == pygame.JOYBUTTONUP:
#     #     print("Joystick button released.")

#     # joystick = pygame.joystick.Joystick(0)
#     # joystick.init()

#     # # Usually axis run in pairs, up/down for one, and left/right for
#     # # the other.
#     # axes = []
#     # buttons = []

#     # for a in range(joystick.get_numaxes()):
#     #   axes.append(joystick.get_axis(a))

#     # for b in range(joystick.get_numbuttons()):
#     #   buttons.append(bool(joystick.get_button(b)))
#     if keyboard.is_pressed('q'):  # if key 'q' is pressed 
#         print('You Pressed A Key!')
#         break  # finishing the loop

#     dat = messaging.new_message('testJoystick')
#     # dat.testJoystick.axes = axes
#     # dat.testJoystick.buttons = buttons
#     keyboard_sock.send(dat.to_bytes())

#     # Limit to 10 frames per second
#     sleep(0.1)

# if __name__ == "__main__":
#   keyboard_thread()
