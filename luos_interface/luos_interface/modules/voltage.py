from luos_msgs.msg import FloatChange
from std_msgs.msg import Float32
from .serializers import serializeFloat32, serializeFloatChange
from .deserializers import deserializeFloat32
from .generic import LuosGenericPublisher

class LuosVoltagePublisher(LuosGenericPublisher):
    def __init__(self, node, module, rate):
        variables = {
            "volt": {"type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      "read": True, "write": True},
            "threshold": {"type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      "read": True, "write": True},
        }
        events = {
            "changed": {"type": FloatChange, "serialize": serializeFloatChange},
            "filter_changed": {"type": FloatChange, "serialize": serializeFloatChange},
        }
        aggregates = {}
        super(LuosVoltagePublisher, self).__init__(node, module, rate, variables, events, aggregates)
