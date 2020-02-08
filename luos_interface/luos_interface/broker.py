import rclpy
from os import listdir
from os.path import isdir, join
from pyluos import Robot
from serial import SerialException

class LuosBroker(rclpy.node.Node):
    RATE_HZ = 10
    def __init__(self):
        super().__init__("luos_broker")
        self._port = ""
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

    def _timer_callback(self):
        for module in self._robot.modules:
            if module.type == "Gate":
                pass
            else:
                self._log.warn("Ignoring unkown Luos module with type '{}'".format(module.type))

    def run(self):
        self._log = self.get_logger()
        self._publisher = self.create_publisher(String, 'topic', 10)
        if self.autoconnect():
            self._log.info("Found modules {}".format(self._robot.modules))
            self._timer = self.create_timer(1./self.RATE_HZ, self._timer_callback)
        else:
            self._log.error("No Luos module found")


def main(): return LuosBroker().run()
if __name__ == '__main__': main()
