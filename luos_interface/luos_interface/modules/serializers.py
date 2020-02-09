from std_msgs.msg import Float32, ColorRGBA
from luos_msgs.msg import State

def serializeColor(module, data):
    return ColorRGBA(
        r=float('nan') if data is None else data[0],
        g=float('nan') if data is None else data[1],
        b=float('nan') if data is None else data[2],
        a=float('nan'),                
    )

def serializeFloat32(module, data):
    return Float32(data=data)

def serializeState(module, event):
    return State(
        #common=LuosCommonMessageSerializer.luos_to_ros(module),
        old_value=event.old_value,
        new_value=event.new_value
    )