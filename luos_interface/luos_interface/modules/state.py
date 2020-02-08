from luos_msgs.msg import Common, State

class LuosStatePublisher(object):
    QUEUE_SIZE = 10
    def __init__(self, node, module):
        self._node = node
        self._publishers = {}

        def serialize(module, event):
            return State(
                #common=LuosCommonMessageSerializer.luos_to_ros(module),
                old_value=event.old_value,
                new_value=event.new_value
            )

        for event in ["changed", "falling", "rising"]:
            topic = "/".join([module.alias, "events", event])
            self._publishers[event] = self._node.create_publisher(State, topic, self.QUEUE_SIZE)
            module.add_callback(event, lambda e: self._publishers[e.name].publish(serialize(module, e)))