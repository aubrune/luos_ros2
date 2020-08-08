from luos_msgs.msg import FloatChange
from std_msgs.msg import Float32
from .serializers import serializeFloat32, serializeFloatChange
from .deserializers import deserializeFloat32
from .generic import LuosGenericPublisher

class LuosServoMotorPublisher(LuosGenericPublisher):
    def __init__(self, node, module, rate):
        variables = {
            "rot_position": {"read_type": Float32, "write_type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      },
            "max_angle": {"read_type": Float32, "write_type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      },
            "min_pulse": {"read_type": Float32, "write_type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      },
            "max_pulse": {"read_type": Float32, "write_type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      },
        }
        events = {}
        aggregates = {}
        super(LuosServoMotorPublisher, self).__init__(node, module, rate, variables, events, aggregates)
