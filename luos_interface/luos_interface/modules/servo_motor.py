from luos_msgs.msg import FloatChange
from std_msgs.msg import Float32
from .serializers import serializeFloat32, serializeFloatChange
from .deserializers import deserializeFloat32
from .generic import LuosGenericPublisher

class LuosServoMotorPublisher(LuosGenericPublisher):
    def __init__(self, node, module, rate):
        variables = {
            "rot_position": {"type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      "read": True, "write": False},
            "max_angle": {"type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      "read": True, "write": False},
            "min_pulse": {"type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      "read": True, "write": False},
            "max_pulse": {"type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      "read": True, "write": False},
        }
        events = {}
        aggregates = {}
        super(LuosServoMotorPublisher, self).__init__(node, module, rate, variables, events, aggregates)
