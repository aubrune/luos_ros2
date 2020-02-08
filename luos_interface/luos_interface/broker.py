import rclpy
from rclpy.node import Node
from os import listdir
from os.path import isdir, join
from pyluos import Robot
from serial import SerialException
from .modules import make_module_interface_factory

class LuosBroker(Node):
    RATE_HZ = 10
    def __init__(self):
        super(LuosBroker, self).__init__("luos_broker")
        self._port = ""
        self._robot = None
        self._module_interfaces = {}
        self._log = self.get_logger()
        if self.autoconnect():
            self._log.info("Found modules {}".format(self._robot.modules))
            for module in self._robot.modules:
                self._module_interfaces[module.alias] = make_module_interface_factory(self, module)
        else:
            self._log.error("No Luos module found")

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

    def close(self):
        if self._robot is not None:
            self._robot.close()

def main():
    rclpy.init()
    broker = LuosBroker()
    try:
        rclpy.spin(broker)
    except KeyboardInterrupt:
        pass
    finally:
        broker.close()

if __name__ == '__main__': main()
