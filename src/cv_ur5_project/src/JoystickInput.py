import pygame

# Initialize Pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Get the number of available joysticks
joystick_count = pygame.joystick.get_count()
print(joystick_count)
if joystick_count == 0:
    print("No joysticks found.")
else:
    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("Joystick Name:", joystick.get_name())

    # Main loop
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYAXISMOTION:
                # Handle axis motion event
                print("Axis", event.axis, "Value:", event.value)
            elif event.type == pygame.JOYBUTTONDOWN:
                # Handle button press event
                print("Button", event.button, "Pressed")

# Quit Pygame
pygame.quit()
