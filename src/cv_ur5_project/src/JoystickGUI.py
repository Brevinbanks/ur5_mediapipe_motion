import evdev

# Define the keyboard device
keyboard = evdev.InputDevice('/dev/input/event0')  # You may need to adjust the device path

# Define the key codes for the keys of interest
KEY_W = 17
KEY_A = 30
KEY_S = 31
KEY_D = 32
KEY_UP = 103
KEY_DOWN = 108
KEY_LEFT = 105
KEY_RIGHT = 106
KEY_E = 18
KEY_C = 46
KEY_SPACE = 57

# Dictionary to map key codes to key names
key_names = {
    KEY_W: 'W',
    KEY_A: 'A',
    KEY_S: 'S',
    KEY_D: 'D',
    KEY_UP: 'Up',
    KEY_DOWN: 'Down',
    KEY_LEFT: 'Left',
    KEY_RIGHT: 'Right',
    KEY_E: 'E',
    KEY_C: 'C',
    KEY_SPACE: 'Space'
}

# Listen for keyboard events
print("Listening for keyboard events. Press 'Q' to quit.")
for event in keyboard.read_loop():
    if event.type == evdev.ecodes.EV_KEY:
        key_code = event.code
        if key_code in key_names:
            key_name = key_names[key_code]
            if event.value == 1:  # Key press event
                print("Key pressed:", key_name)
            elif event.value == 0:  # Key release event
                print("Key released:", key_name)
    if event.type == evdev.ecodes.EV_KEY and event.code == evdev.ecodes.KEY_Q:/
        break
