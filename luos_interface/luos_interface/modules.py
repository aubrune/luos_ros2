from luos_msgs.msg import Common, Gate, Imu

make = {
    'Imu': LuosImuPublisher,
    'Gate': LuosGatePublisher,
    'State': LuosStatePublisher,
    'Color': LuosColorPublisher,
}

def make_module_interface_factory(module):
    if module.type in make:
        return make[module.type](module)
    else:
        print(ValueError("Luos module type '{}' is unknown".format(type(module.type))))
        #TODO raise ValueError("Luos module type '{}' is unknown".format(type(module.type)))

class LuosStatePublisher(object):
    QUEUE_SIZE = 10
    def __init__(self, node):
        self._node = node
        self._publishers = {}

    def register_events(module):
        def serialize(module, event):
            return State(
                common=LuosCommonMessageSerializer.luos_to_ros(module),
                old_value=event.old_value,
                new_value=event.new_value
            )

        for event in ["changed", "falling", "raising"]:
            topic = "/".join(module.alias, event)
            self._publishers[event] = self._node.create_publisher(State, topic, self.QUEUE_SIZE)
            module.register_events(event, lambda event: self._publishers[event].publish(serialize(module, event)))