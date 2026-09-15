import pygame
import sys

# Initialize pygame and the joystick module
pygame.init()
pygame.joystick.init()

# Check for connected controllers
if pygame.joystick.get_count() == 0:
    print("No controller found. Please connect your controller via Bluetooth.")
    sys.exit()

# Initialize the first connected controller
controller = pygame.joystick.Joystick(0)
#controller.init() will not work on future versions of pygame

print("=======================================")
print(f" Connected: {controller.get_name()}")
print("=======================================")
print("Status: READY")
print("Instructions: Move sticks or press buttons to test. Press 'B' (or 'Circle') to exit.\n")

# Deadzone to ignore slight stick drift
DEADZONE = 0.15

try:
    while True:
        # Process events from pygame
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                sys.exit()

            # Handle Analog Stick Movement (Translation & Rotation)
            elif event.type == pygame.JOYAXISMOTION:
                axis = event.axis
                value = event.value

                # Ignore inputs inside the deadzone
                if abs(value) > DEADZONE:
                    # Axis 0/1: Left Stick (X/Y) | Axis 2/3 or 3/4: Right Stick (Yaw/Pitch)
                    # Note: Trigger axes vary depending on the controller mode (X-input vs D-input)
                    print(f"Left Stick {axis} moved to {value:.2f}")

            # Handle Button Presses (System Commands / E-Stop / Roll)
            elif event.type == pygame.JOYBUTTONDOWN:
                button = event.button
                print(f"Button {button} pressed")

                # In X-Input mode, Button 1 is usually 'B', used here as an E-Stop / Exit
                if button == 1:
                    print("E-STOP TRIGGERED / Exiting POSE control interface...")
                    pygame.quit()
                    sys.exit()

        # Small delay to prevent the loop from maxing out your CPU
        pygame.time.wait(10)

except KeyboardInterrupt:
    print("Exiting...")
    pygame.quit()
