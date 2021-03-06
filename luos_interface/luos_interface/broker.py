import rclpy
from rclpy.node import Node
from os import listdir
from os.path import isdir, join
from pyluos import Device
from serial import SerialException
from rcl_interfaces.srv import GetParameters
from .modules import make_module_interface_factory
from .utils.serial import get_available_ports


class LuosBroker(Node):
    DEFAULT_RATE_HZ = 30
    def __init__(self):
        super(LuosBroker, self).__init__("luos_broker")
        self._device = None
        self._module_interfaces = {}
        self._log = self.get_logger()
        self.declare_parameter("device")
        self.declare_parameter("rate")
        device = self.get_parameter("device").get_parameter_value().string_value
        rate = self.get_parameter("rate").get_parameter_value().integer_value
        self._rate = rate if rate != 0 else self.DEFAULT_RATE_HZ
        self.connected = self.connect(device)

    @property
    def num_modules(self):
        return len(self._device.modules) if self._device is not None else 0

    def connect(self, device=""):
        # Connect either to a specified device or autoconnect if no device is specified
        if device in ["auto", "", None]:
            self._log.info("Automatic search for a Luos device")
            success = self.autoconnect()
        else:
            self._log.info("Manual connection to a Luos device")
            success = self._connect(device)
        if not success:
            self._log.error("Can't connect to any Luos device")
        return success

    def _connect(self, device):
        self._log.info('Connecting to {}...'.format(device))
        try:
            self._device = Device(device)
        except SerialException as e:
            self._log.error(repr(e))
            return False
        except ValueError as e:
            self._log.error(repr(e))
            return False
        else:
            self._log.info("Broker found {} modules:\r\n{}".format(self.num_modules, self._device.modules))
            for module in self._device.modules:
                self._module_interfaces[module.alias] = make_module_interface_factory(self, module, self._rate)
            return True

    def autoconnect(self):
        ports = get_available_ports()
        for device in ports:
            if self._connect(device):
                param = rclpy.parameter.Parameter("device", rclpy.Parameter.Type.STRING, device)
                self.set_parameters([param])
                return True
        return False

    def close(self):
        if self._device is not None:
            self._device.close()

def main():
    rclpy.init()
    broker = LuosBroker()
    if not broker.connected: return
    broker._log.info("Broker is connected to {} Luos modules at {}Hz!".format(broker.num_modules, broker._rate))
    try:
        rclpy.spin(broker)
    except KeyboardInterrupt:
        pass
    finally:
        broker.close()

if __name__ == '__main__': main()
