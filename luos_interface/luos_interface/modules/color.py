from std_msgs.msg import Float32, ColorRGBA
from .serializers import serializeColor, serializeFloat32
from .generic import LuosGenericPublisher

class LuosColorPublisher(LuosGenericPublisher):
    def __init__(self, node, module):
        variables = {
            "color": {"type": ColorRGBA, "serialize": serializeColor},
            "time": {"type": Float32, "serialize": serializeFloat32}
        }
        events = {}
        super(LuosColorPublisher, self).__init__(node, module, variables, events)
