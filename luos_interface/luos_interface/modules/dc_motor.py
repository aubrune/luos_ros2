from std_msgs.msg import Float32
from .serializers import serializeFloat32
from .deserializers import deserializeFloat32
from .generic import LuosGenericPublisher

class LuosDcMotorPublisher(LuosGenericPublisher):
    def __init__(self, node, module, rate):
        variables = {
            "power_ratio": {"type": Float32,
                      "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                      "read": True, "write": False},
        }
        events = {}
        aggregates = {}
        super(LuosDcMotorPublisher, self).__init__(node, module, rate, variables, events, aggregates)
