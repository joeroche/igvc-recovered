
#left joystick x axis = 0
#left joystick y axis = 1
#right joystick x axis = 2
#right joystick y axis = 3

#joystick positive y axis is down, negative y axis is up 
#joystick positive x axis is right, negative x axis is left

import pygame
import time

deadzone = 0.1

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count()==0:
    print("no controller found")
    pygame.quit()
    quit()

joystick = pygame.joystick.Joystick(0)  
joystick.init()

print(f"Joystick name: {joystick.get_name()}")
print(f"Number of axes: {joystick.get_numaxes()}")
print(f"Number of buttons: {joystick.get_numbuttons()}")

try:
    while True:
        # Process Pygame events
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                axis = joystick.get_axis(2)
                if abs(axis) > deadzone: 
                    if axis > 0.995: #positive y axis never seems to reach 1.0, so this if statement does that
                        axis = 1.0
                    print(axis)
                # for i in range(joystick.get_numaxes()):
                #     axis = joystick.get_axis(i) + 1
                #     if abs(axis) > deadzone:
                #         print(f"Axis {i} value: {axis:.2f}")
            elif event.type == pygame.JOYBUTTONDOWN:
                for i in range(joystick.get_numbuttons()):
                    button = joystick.get_button(i)
                    if button:
                        print(f"Button {i} pressed")
            elif event.type == pygame.JOYBUTTONUP:
                for i in range(joystick.get_numbuttons()):
                    button = joystick.get_button(i)
                    if not button:
                        print(f"Button {i} released")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Quit Pygame
    pygame.quit()