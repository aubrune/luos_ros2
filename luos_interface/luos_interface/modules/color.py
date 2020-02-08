from luos_msgs.msg import Common
from std_msgs.msg import Float32, ColorRGBA

def serializeColor(module, data):
    if data is None:
        return ColorRGBA()
    else:
        return ColorRGBA(
            #common=LuosCommonMessageSerializer.luos_to_ros(module),
            r=data[0],
            g=data[1],
            b=data[2],
            a=float('nan'),                
        )

def serializeFloat32(module, data):
    return Float32(data=data)

class LuosColorPublisher(object):
    QUEUE_SIZE = 10
    RATE_HZ = 20
    def __init__(self, node, module):
        self._node = node
        self._module = module
        self.variables = {
            "color": {"type": ColorRGBA, "serialize": serializeColor},
            "time": {"type": Float32, "serialize": serializeFloat32}
        }

        self._publishers = {}
        for variable in self.variables:
            type = self.variables[variable]["type"]
            topic = "/".join([module.alias, "variables", variable, "read"])
            self._publishers[variable] = {
                "topic": topic,
                "type": type,
                "pub": self._node.create_publisher(type, topic, self.QUEUE_SIZE)
            }
        
        self.timer = self._node.create_timer(1./self.RATE_HZ, self._timer_callback)
    
    def _timer_callback(self):
        for variable, info in self.variables.items():
            print("MODULE", self._module, "variable", variable)
            serialize = info["serialize"]
            self._publishers[variable]["pub"].publish(serialize(self._module, getattr(self._module, variable)))
            print('END')

