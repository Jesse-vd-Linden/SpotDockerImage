import pygame
from time import sleep
pygame.init()
pygame.joystick.init()

joystick_count = pygame.joystick.get_count()
if joystick_count > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
else:
    print("No joystick detected")
    exit()
            
print(pygame.CONTROLLER_BUTTON_DPAD_DOWN)
try:
    while True:
        for event in pygame.event.get():
            try:
                if event.button == 12:
                    print("Joystick button pressed down")
                elif event.button == 11:
                    print("Joystick button pressed up")
                elif event.button == 13:
                    print("Joystick button pressed left")
                if event.button == 14:
                    print("Joystick button pressed right")
            except:
                pass
                
            
        sleep(1)
        # break
except KeyboardInterrupt:
    pass


pygame.quit()