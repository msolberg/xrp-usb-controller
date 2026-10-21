import unittest

from xrp_usb_controller import XRPController

class XRPControllerTests(unittest.TestCase):
    """ Test the XRP Controller - only runs commands with no motion """

    @classmethod
    def setUpClass(cls):
        """ Set up the connection to the robot before running tests. """
        cls.bot = XRPController(port="/dev/ttyACM0")

    @classmethod
    def tearDownClass(cls):
        """ Close the connection to the bot after running tests. """
        cls.bot.led_off()
        cls.bot.close()

    def test_connection(self):
        conn = self.bot.sync_connection()
        self.assertTrue(conn)

    def test_ping(self):
        ping = self.bot.ping()
        self.assertTrue(ping)

    def test_send_command(self):
        badcommand = self.bot.send_command("CHICKEN")
        goodcommand = self.bot.send_command("PING")
        self.assertFalse(goodcommand.startswith("ACK:CHICKEN"))
        self.assertTrue(goodcommand.startswith("ACK:PING"))

    def test_led(self):
        resp = self.bot.set_rgb(0, 255, 0)
        self.assertTrue(resp.startswith("ACK:RGB,0,255,0"))
        resp = self.bot.led_off()
        self.assertTrue(resp.startswith("ACK:OFF"))

    def test_servo(self):
        resp = self.bot.free_servo(1)
        self.assertTrue(resp.startswith("ACK:SERVO,1,FREE"))
        resp = self.bot.free_servo(2)
        self.assertTrue(resp.startswith("ACK:SERVO,2,FREE"))
    
    def test_rangefinder(self):
        resp = self.bot.read_rangefinder()
        self.assertTrue(isinstance(resp, float))

    def test_imu(self):
        d = self.bot.read_imu()
        self.assertTrue(isinstance(d, dict))
        self.assertTrue(isinstance(d['heading'], float))
        self.assertTrue(isinstance(d['yaw'], float))
        self.assertTrue(isinstance(d['pitch'], float))
        self.assertTrue(isinstance(d['roll'], float))
        self.assertTrue(isinstance(d['acc_x'], float))
        self.assertTrue(isinstance(d['acc_y'], float))
        self.assertTrue(isinstance(d['acc_z'], float))
        self.assertTrue(isinstance(d['gyro_x'], float))
        self.assertTrue(isinstance(d['gyro_y'], float))
        self.assertTrue(isinstance(d['gyro_z'], float))
        self.assertTrue(isinstance(d['temperature'], float))

    def test_drivetrain(self):
        effort = self.bot.drive_effort(0, 0)
        self.assertTrue(effort.startswith("ACK:DRIVE,EFFORT"))
        speed = self.bot.drive_speed(0, 0)
        self.assertTrue(speed.startswith("ACK:DRIVE,SPEED"))
        arcade = self.bot.drive_arcade(0, 0)
        self.assertTrue(arcade.startswith("ACK:DRIVE,ARCADE"))
        straight = self.bot.drive_straight(0)
        self.assertTrue(straight.startswith("ACK:DRIVE,STRAIGHT"))
        turn = self.bot.drive_turn(0)
        self.assertTrue(turn.startswith("ACK:DRIVE,TURN"))
        stop = self.bot.drive_stop()
        self.assertTrue(stop.startswith("ACK:DRIVE,STOP"))

    


