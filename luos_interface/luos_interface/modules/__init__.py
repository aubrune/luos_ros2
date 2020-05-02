from .state import LuosStatePublisher
from .color import LuosColorPublisher
from .imu import LuosImuPublisher

_make = {
    'State': LuosStatePublisher,
    'Color': LuosColorPublisher,
    'Imu': LuosImuPublisher,
}

def make_module_interface_factory(node, module, rate):
    if module.type in _make:
        return _make[module.type](node, module, rate)
    elif module.type != 'Gate':
        # Gate has no publisher or subscriber
        node.get_logger().warn("Luos module type '{}' is unknown to luos_interface and will be ignored".format(module.type))
