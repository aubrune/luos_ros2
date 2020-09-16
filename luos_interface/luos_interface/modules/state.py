from luos_msgs.msg import BoolChange
from std_msgs.msg import Bool
from .serializers import serializeBoolChange, serializeBool
from .deserializers import deserializeBool
from .generic import LuosGenericPublisher

class LuosStatePublisher(LuosGenericPublisher):
    def __init__(self, node, module, rate):
        variables = {
            "state": {
                "read_type": Bool, "write_type": Bool,
                "serialize": serializeBool, "deserialize": deserializeBool,
                },
        }
        events = {
            "released": {"type": BoolChange, "serialize": serializeBoolChange},
            "falling": {"type": BoolChange, "serialize": serializeBoolChange},
            "changed": {"type": BoolChange, "serialize": serializeBoolChange}
        }
        aggregates = {}
        super(LuosStatePublisher, self).__init__(node, module, rate, variables, events, aggregates)
