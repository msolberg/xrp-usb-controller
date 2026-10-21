#!/usr/bin/env python

import sys
import uselect
import asyncio
import micropython
import time
from XRPLib.defaults import Board, drivetrain, imu
from XRPLib.rangefinder import Rangefinder
from XRPLib.servo import Servo

_VERSION = '0.3.0'

# Hardware initialization
board = Board.get_default_board()
servo_one = Servo.get_default_servo(1)
servo_two = Servo.get_default_servo(2)
rf = Rangefinder()

# Prevent incoming 0x03 (Ctrl+C) bytes from killing the event loop
micropython.kbd_intr(-1)

class XRPTelemetryCache:
    def __init__(self):
        # Rangefinder state
        self.distance_cm: float = 0.0
        self.distance_last_update: int = 0
        
        # IMU state
        self.pitch: float = 0.0
        self.yaw = float = 0.0
        self.heading: float = 0.0
        self.roll: float = 0.0
        self.acc_x: int = 0 
        self.acc_y: int = 0 
        self.acc_z: int = 0 
        self.gyro_x: int = 0 
        self.gyro_y: int = 0 
        self.gyro_z: int = 0 
        self.temperature: int = 0
        self.imu_last_update: int = 0

    async def _poll_rangefinder(self, interval_ms: int = 50):
        """Dedicated rangefinder task (50ms / 20 Hz)."""
        while True:
            # 30ms max timeout happens inside distance() non-blockingly to other tasks
            self.distance_cm = rf.distance()
            self.distance_last_update = time.ticks_ms()
            await asyncio.sleep_ms(interval_ms)

    async def _poll_imu(self, interval_ms: int = 15):
        """Dedicated IMU task (~15ms / ~66 Hz)."""
        while True:
            self.pitch = imu.get_pitch()
            self.yaw = imu.get_yaw()
            self.heading = imu.get_heading()
            self.roll = imu.get_roll()
            (self.accel_x, self.accel_y, self.accel_z) = imu.get_acc_rates()
            (self.gyro_x, self.gyro_y, self.gyro_z) = imu.get_gyro_rates()
            self.temperature = imu.temperature()
            self.imu_last_update = time.ticks_ms()
            await asyncio.sleep_ms(interval_ms)

    def get_distance(self) -> float:
        """Returns distance from cache if fresh"""
        now = time.ticks_ms()
        age = time.ticks_diff(now, self.distance_last_update)
        if self.distance_last_update !=0 and age < 100:
            return self.distance_cm
        # Otherwise, refresh the value and return it
        self.distance_cm = rf.get_distance()
        self.distance_last_update = time.ticks_ms()
        return self.distance_cm

    def get_pitch(self) -> float:
        """Returns pitch from cache if fresh"""
        now = time.ticks_ms()
        age = time.ticks_diff(now, self.imu_last_update)
        if self.imu_last_update !=0 and age < 100:
            return self.pitch
        # Otherwise, refresh the value and return it
        self.pitch = imu.get_pitch()
        self.imu_last_update = time.ticks_ms()
        return self.pitch

    def get_yaw(self) -> float:
        """Returns yaw from cache if fresh"""
        now = time.ticks_ms()
        age = time.ticks_diff(now, self.imu_last_update)
        if self.imu_last_update !=0 and age < 100:
            return self.yaw
        # Otherwise, refresh the value and return it
        self.yaw = imu.get_yaw()
        self.imu_last_update = time.ticks_ms()
        return self.pitch

    def get_heading(self) -> float:
        """Returns heading from cache if fresh"""
        now = time.ticks_ms()
        age = time.ticks_diff(now, self.imu_last_update)
        if self.imu_last_update !=0 and age < 100:
            return self.heading
        # Otherwise, refresh the value and return it
        self.heading = imu.get_heading()
        self.imu_last_update = time.ticks_ms()
        return self.heading
    
    def get_roll(self) -> float:
        """Returns roll from cache if fresh"""
        now = time.ticks_ms()
        age = time.ticks_diff(now, self.imu_last_update)
        if self.imu_last_update !=0 and age < 100:
            return self.roll
        # Otherwise, refresh the value and return it
        self.roll = imu.get_roll()
        self.imu_last_update = time.ticks_ms()
        return self.roll

    def get_acc_rates(self) -> tuple[float]:
        """Returns the acceleration rates for x,y,z"""
        now = time.ticks_ms()
        age = time.ticks_diff(now, self.imu_last_update)
        if self.imu_last_update !=0 and age < 100:
            return (self.acc_x, self.acc_y, self.acc_z)
        # Otherwise, refresh the value and return it
        (self.acc_x, self.acc_y, self.acc_z) = imu.get_acc_rates()
        return (self.acc_x, self.acc_y, self.acc_z)

    def get_gyro_rates(self) -> tuple[float]:
        """Returns the acceleration rates for x,y,z"""
        now = time.ticks_ms()
        age = time.ticks_diff(now, self.imu_last_update)
        if self.imu_last_update !=0 and age < 100:
            return (self.gyro_x, self.gyro_y, self.gyro_z)
        # Otherwise, refresh the value and return it
        (self.gyro_x, self.gyro_y, self.gyro_z) = imu.get_gyro_rates()
        return (self.gyro_x, self.gyro_y, self.gyro_z)

    def get_temperature(self) -> float:
        """Returns the temperature in degrees celcius"""
        now = time.ticks_ms()
        age = time.ticks_diff(now, self.imu_last_update)
        if self.imu_last_update !=0 and age < 100:
            return (self.temperature)
        # Otherwise, refresh the value and return it
        self.temperature = imu.temperature()
        return self.temperature
    
    def start(self):
        """Schedules both polling routines on the event loop."""
        asyncio.create_task(self._poll_rangefinder(interval_ms=50))
        asyncio.create_task(self._poll_imu(interval_ms=15))

# Initialize the cache
cache = XRPTelemetryCache()

# -------------------------------------------------------------
# Command Dispatch Table
# -------------------------------------------------------------
COMMANDS = {}

def command(name):
    """Decorator to register a command handler."""
    def decorator(func):
        COMMANDS[name.upper()] = func
        return func
    return decorator

@command("VERSION")
def get_version():
    """Return the current version of the control program"""
    return f"{_VERSION}"

@command("STOP")
def cmd_stop():
    """Terminate the program cleanly."""
    return False

@command("RGB")
def cmd_rgb(r, g, b):
    """Control LED RGB values (or blink indicator)."""
    r, g, b = int(r), int(g), int(b)
    board.led_blink(1)
    return f"{r},{g},{b}"

@command("OFF")
def cmd_off():
    """Turn off the board LED."""
    board.led_off()
    return "OK"

@command("SERVO")
def cmd_servo(servo_num, angle):
    """Set servo angle: SERVO,<1|2>,<angle> or free servo: SERVO,<1|2>,FREE"""
    s = servo_one if str(servo_num) == "1" else servo_two if str(servo_num) == "2" else None
    if s is None:
        raise ValueError(f"Invalid servo ID: {servo_num}")
    if str(angle).upper() == "FREE":
        s.free()
        return f"{servo_num},FREE"
    angle_val = float(angle)
    s.set_angle(angle_val)
    return f"{servo_num},{angle_val}"

@command("PING")
def cmd_ping():
    """Connectivity heartbeat / handshake."""
    return "PONG"

@command("RF")
def cmd_rangefinder():
    """Query rangefinder distance in cm."""
    dist = cache.get_distance()
    return f"{dist:.2f}"

@command("IMU")
def cmd_imu():
    """Query the IMU."""
    heading = cache.get_heading()
    yaw = cache.get_yaw()
    pitch = cache.get_pitch()
    roll = cache.get_roll()
    (acc_x, acc_y, acc_z) = cache.get_acc_rates()
    (gyro_x, gyro_y, gyro_z) = cache.get_gyro_rates()
    temperature = cache.get_temperature()
    return f"{heading:.2f},{yaw:.2f},{pitch:.2f},{roll:.2f},{acc_x:.2f},{acc_y:.2f},{acc_z:.2f},{gyro_x:.2f},{gyro_y:.2f},{gyro_z:.2f},{temperature:.2f}"

@command("DRIVE")
def cmd_drivetrain(action, *args):
    """Control drivetrain:
    DRIVE,STOP
    DRIVE,EFFORT,<left>,<right>
    DRIVE,SPEED,<left>,<right>
    DRIVE,ARCADE,<straight>,<turn>
    DRIVE,STRAIGHT,<distance>[,<effort>]
    DRIVE,TURN,<degrees>[,<effort>]
    """
    action = action.upper()
    if action == "STOP":
        drivetrain.stop()
    elif action == "EFFORT":
        if len(args) != 2:
            raise ValueError("EFFORT requires <left>,<right>")
        drivetrain.set_effort(float(args[0]), float(args[1]))
    elif action == "SPEED":
        if len(args) != 2:
            raise ValueError("SPEED requires <left>,<right>")
        drivetrain.set_speed(float(args[0]), float(args[1]))
    elif action == "ARCADE":
        if len(args) != 2:
            raise ValueError("ARCADE requires <straight>,<turn>")
        drivetrain.arcade(float(args[0]), float(args[1]))
    elif action == "STRAIGHT":
        if len(args) < 1:
            raise ValueError("STRAIGHT requires <distance>[,<effort>]")
        dist = float(args[0])
        effort = float(args[1]) if len(args) > 1 else 0.5
        drivetrain.straight(dist, max_effort=effort)
    elif action == "TURN":
        if len(args) < 1:
            raise ValueError("TURN requires <degrees>[,<effort>]")
        deg = float(args[0])
        effort = float(args[1]) if len(args) > 1 else 0.5
        drivetrain.turn(deg, max_effort=effort)
    else:
        raise ValueError(f"Unknown drive action: {action}")
    return action

async def handle_command(line: str) -> bool:
    """Dispatches line to registered commands. Returns False if STOP is received."""
    line = line.strip()
    if not line:
        return True

    parts = line.split(",")
    cmd = parts[0].upper()
    args = parts[1:]

    handler = COMMANDS.get(cmd)
    if not handler:
        print(f"ERR:UNKNOWN_CMD:{cmd}")
        return True

    try:
        result = handler(*args)
        if result is False:
            print("ACK:STOP")
            return False

        if result is not None:
            if result == "OK":
                print(f"ACK:{cmd}")
            else:
                print(f"ACK:{cmd},{result}")
        else:
            print(f"ACK:{cmd}")
    except TypeError:
        print(f"ERR:BAD_ARGS:{cmd}")
    except ValueError as e:
        print(f"ERR:INVALID_VAL:{e}")
    except Exception as e:
        print(f"ERR:{cmd}:{e}")

    return True

async def usb_serial_listener():
    """Asynchronously reads incoming lines from USB Serial (stdin)."""
    sreader = asyncio.StreamReader(sys.stdin)
    print("READY")

    while True:
        # Await incoming bytes without blocking other async tasks
        raw_data = await sreader.readline()
        if raw_data:
            line_str = raw_data.decode("utf-8") if isinstance(raw_data, bytes) else raw_data
            if not await handle_command(line_str):
                break

async def robot_background_task():
    """Example concurrent task (e.g., telemetry heartbeat, sensor polling)."""
    try:
        while True:
            # Simulates non-blocking background logic running every 500ms
            await asyncio.sleep_ms(500)
    except asyncio.CancelledError:
        pass

async def main():
    cache.start()
    
    # Set initial visual ready state (dim blue)
    board.led_on()

    # Run the USB listener and robot tasks concurrently
    bg_task = asyncio.create_task(robot_background_task())
    try:
        await usb_serial_listener()
    finally:
        bg_task.cancel()
        try:
            await bg_task
        except asyncio.CancelledError:
            pass
        drivetrain.stop()
        board.led_off()
        micropython.kbd_intr(3)  # Restore Ctrl+C handling in the REPL

# Start the asyncio event loop
asyncio.run(main())

