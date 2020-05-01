from std_msgs.msg import Float32, ColorRGBA
from .serializers import serializeColor, serializeFloat32
from .deserializers import deserializeColor, deserializeFloat32
from .generic import LuosGenericPublisher

class LuosColorPublisher(LuosGenericPublisher):
    def __init__(self, node, module):
        variables = {
            "color": {"type": ColorRGBA,
                      "serialize": serializeColor, "deserialize": deserializeColor,
                      "read": True, "write": True},
            "time": {"type": Float32,
                     "serialize": serializeFloat32, "deserialize": deserializeFloat32,
                     "read": True, "write": True}
        }
        events = {}
        aggregates = {}
        super(LuosColorPublisher, self).__init__(node, module, variables, events, aggregates)
