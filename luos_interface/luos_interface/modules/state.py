from luos_msgs.msg import State
from std_msgs.msg import Bool
from .serializers import serializeState, serializeBool
from .deserializers import deserializeBool
from .generic import LuosGenericPublisher

class LuosStatePublisher(LuosGenericPublisher):
    def __init__(self, node, module, rate):
        variables = {
            "state": {"type": Bool,
                      "serialize": serializeBool, "deserialize": deserializeBool,
                      "read": True, "write": True},
        }
        events = {
            "rising": {"type": State, "serialize": serializeState},
            "falling": {"type": State, "serialize": serializeState},
            "changed": {"type": State, "serialize": serializeState}
        }
        aggregates = {}
        super(LuosStatePublisher, self).__init__(node, module, rate, variables, events, aggregates)
