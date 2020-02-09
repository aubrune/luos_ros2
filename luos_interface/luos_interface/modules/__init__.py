from .state import LuosStatePublisher
from .color import LuosColorPublisher
from .imu import LuosImuPublisher

_make = {
    'State': LuosStatePublisher,
    'Color': LuosColorPublisher,
    'Imu': LuosImuPublisher,
}

def make_module_interface_factory(node, module):
    if module.type in _make:
        return _make[module.type](node, module)
    else:
        print(ValueError("Luos module type '{}' is unknown".format(module.type)))
        #TODO raise ValueError("Luos module type '{}' is unknown".format(type(module.type)))
