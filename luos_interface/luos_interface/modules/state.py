from luos_msgs.msg import State
from .serializers import serializeState
from .generic import LuosGenericPublisher

class LuosStatePublisher(LuosGenericPublisher):
    def __init__(self, node, module):
        variables = {}
        events = {
            "rising": {"type": State, "serialize": serializeState},
            "falling": {"type": State, "serialize": serializeState},
            "changed": {"type": State, "serialize": serializeState}
        }
        aggregates = {}
        super(LuosStatePublisher, self).__init__(node, module, variables, events, aggregates)
