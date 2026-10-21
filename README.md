# xrp-usb-controller
Library for controlling an XRP Robot over a USB connection

The repository provides a client-server architecture for controlling the XRP robot from a host computer (e.g., PC or SBC) over USB serial:

* **[`usb_control.py`](src/xrp_usb_controller/usb_control.py)**: MicroPython RPC server running on the robot's RP2040 microcontroller. It asynchronously listens on USB serial (`sys.stdin`) for commands and drives the motors, reads the ultrasonic rangefinder/IMU, controls servos, and toggles LEDs.
* **[`usb_controller.py`](src/xrp_usb_controller/usb_controller.py)**: Host-side Python library providing the `XRPController` client class. It manages serial connectivity (`/dev/ttyACM0`), handshakes on connect, flushes stale buffers, and sends commands with automatic response handling.

### Deploying the Controller to the Robot

1. **Install [`usb_control.py`](src/xrp_usb_controller/usb_control.py) as the robot's boot script:**
   ```bash
   # Copy usb_control.py to the Pico as main.py so it starts automatically
   mpremote cp usb_control.py :main.py

   # Soft-reboot the board to start the service
   mpremote reset
   ```   
2. **Run the [`usb_controller.py`](src/xrp_usb_controller/usb_controller.py) script interactively to test connectivity:**
   ```bash
   # Activate virtual environment
   source .venv/bin/activate
   
   # Launch the vision & navigation script
   python usb_controller.py
   ```

---

### Using `XRPController` in Python Scripts

Import [`XRPController`](src/xrp_usb_controller/usb_controller.py) and use it as a context manager:

```python
from xrp_usb_controller import XRPController

# Connects to /dev/ttyACM0 by default and performs a handshake
with XRPController() as bot:
    # 1. Check distance with ultrasonic rangefinder (returns cm as float, or None)
    distance = bot.read_rangefinder()
    print(f"Obstacle distance: {distance} cm")

    # 2. Drive motors (-1.0 to 1.0 effort for left and right wheels)
    bot.drive_effort(0.5, 0.5)

    # 3. Stop drivetrain motors
    bot.drive_stop()

    # 4. Control LEDs or servos
    bot.set_rgb(0, 255, 0)
    bot.set_servo(1, 90.0)
```

---
### Testing this module

This module's tests are designed to be run with the robot connected. To run them, install the module locally and then run the test suite:

```
pip install -e .
python -m unittest tests/test_xrp_usb_controller.py -v
```

---
### Troubleshooting

If you get a InvalidVersion exception, then the version of `usb_control.py` on the control board is too old for the controller. Update the code on the control board as documented above.
