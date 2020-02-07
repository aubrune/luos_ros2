import rclpy
from os import listdir
from os.path import isdir, join
from pyluos import Robot
from serial import SerialException

class LuosBroker(object):
    def __init__(self):
        self._port = ""
        self._node = None
        self._log = None
        self._robot = None

    def autoconnect(self):
        # TODO Windows, MacOS
        if isdir("/dev/"):
            # Linux
            devices = listdir("/dev")
            for device in devices:
                if device.startswith("ttyUSB"):
                    path = join("/dev", device)
                    self._log.info('Connecting to {}...'.format(path))
                    try:
                        self._robot = Robot(path)
                    except SerialException as e:
                        self._log.error(repr(e))
                        pass
                    except ValueError as e:
                        self._log.error(repr(e))
                        pass
        return self._robot is not None

    def run(self):
        rclpy.init()
        self._node = rclpy.create_node('luos')
        self._log = self._node.get_logger()
        if self.autoconnect():
            self._log.info("Found modules {}".format(self._robot.modules))
        else:
            self._log.error("No Luos module found")


def main(): return LuosBroker().run()
if __name__ == '__main__': main()
