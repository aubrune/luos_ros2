"""
Deserialization from ROS standard or custom message types and units to Luos modules
There are:
  * module_agnostic deserializers
  * module-dependent deserialiers
"""

#RAD_TO_DEG = 57.29577951308232

def deserializeFloat32(msg):
    return float(msg.data)

def deserializeUInt32(msg):
    return int(msg.data)

def deserializeVector3(msg):
    return [msg.x, msg.y, msg.z]

def deserializeColor(msg):
    return [msg.r, msg.g, msg.b]

