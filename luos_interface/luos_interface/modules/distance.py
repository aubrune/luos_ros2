from luos_msgs.msg import FloatChange
from std_msgs.msg import Float32
from .serializers import serializeFloat32, serializeFloatChange
from .deserializers import deserializeFloat32
from .generic import LuosGenericPublisher

class LuosDistancePublisher(LuosGenericPublisher):
    def __init__(self, node, module, rate):
        variables = {
            "rot_position": {"type": Float32,
                      "serialize": serializeFloat32, "deserialize": None,
                      "read": True, "write": False},
            "threshold": {"type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      "read": True, "write": False},
        }
        events = {
            "changed": {"type": FloatChange, "serialize": serializeFloatChange},
            "filter_changed": {"type": FloatChange, "serialize": serializeFloatChange},
        }
        aggregates = {}
        super(LuosDistancePublisher, self).__init__(node, module, rate, variables, events, aggregates)
