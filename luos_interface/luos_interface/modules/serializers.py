"""
Serialization from Luos modules to ROS standard or custom message types and units
There are:
  * module_agnostic serializers, that only take the data in input
  * module-dependent serialiers, that only take the module in input
"""

from std_msgs.msg import Bool, Float32, UInt32, ColorRGBA
from geometry_msgs.msg import Vector3, Accel
from sensor_msgs.msg import Imu, MagneticField
from luos_msgs.msg import BoolChange, FloatChange
from rclpy.clock import Clock

DEG_TO_RAD=0.017453292519943295
_clock = Clock()

def serializeBool(data):
    return Bool(data=bool(data))

def serializeFloat32(data):
    return Float32(data=float('nan' if data is None else data))

def serializeFloat32DegToRad(data):
    return Float32(data=float('nan' if data is None else data*DEG_TO_RAD))

def serializeUInt32(data):
    return UInt32(data=int(data))

def serializeVector3(data):
    return(Vector3(x=float('nan' if data[0] is None else data[0]),
                   y=float('nan' if data[1] is None else data[1]),
                   z=float('nan' if data[2] is None else data[2])))

def serializeVector3MinMax(data):
    # Vector 3 is used to represent (min, max) couples with x=min, y=max, z=<whatever>
    return(Vector3(x=float('nan' if data[0] is None else data[0]),
                   y=float('nan' if data[1] is None else data[1]),
                   z=float('nan')))

"""
Variable serialization from Color Luos module to ColorRGBA ROS type
"""
def serializeColor(data):
    return ColorRGBA(
        r=float('nan' if data is None else data[0]),
        g=float('nan' if data is None else data[1]),
        b=float('nan' if data is None else data[2]),
        a=float('nan'),                
    )

"""
Aggregated serialization from Imu Luos module to MagneticField ROS type
"""
def serializeMagneticField(module):
    magn = MagneticField()
    magn.header.frame_id = module.alias
    magn.header.stamp = _clock.now().to_msg()
    if module.compass is not None:
        magn.magnetic_field.x = float(module.compass[0])
        magn.magnetic_field.y = float(module.compass[1])
        magn.magnetic_field.z = float(module.compass[2])
    return magn

"""
Aggregated serialization from Imu Luos module to Accel ROS type
"""
def serializeAccel(module):
    accel = Accel()
    if module.acceleration is not None:
        accel.linear.x = float(module.acceleration[0])
        accel.linear.y = float(module.acceleration[1])
        accel.linear.z = float(module.acceleration[2])
    if module.gyro is not None:
        accel.angular.x = float(module.gyro[0]*DEG_TO_RAD)
        accel.angular.y = float(module.gyro[1]*DEG_TO_RAD)
        accel.angular.z = float(module.gyro[2]*DEG_TO_RAD)
    return accel

"""
Aggregated serialization from Imu Luos module to Imu ROS type
"""
def serializeImu(module):
    imu = Imu()
    imu.header.frame_id = module.alias
    imu.header.stamp = _clock.now().to_msg()
    if module.linear_acceleration is not None:
        imu.linear_acceleration.x = float(module.linear_acceleration[0])
        imu.linear_acceleration.y = float(module.linear_acceleration[1])
        imu.linear_acceleration.z = float(module.linear_acceleration[2])
    if module.quaternion is not None:
        imu.orientation.x = float(module.quaternion[0])
        imu.orientation.y = float(module.quaternion[1])
        imu.orientation.z = float(module.quaternion[2])
        imu.orientation.w = float(module.quaternion[3])
    return imu

"""
Event serialization from State Luos module to State ROS type
"""
def serializeBoolChange(module, event):
    return BoolChange(
        old_value=event.old_value,
        new_value=event.new_value
    )

"""
Event serialization 
"""
def serializeFloatChange(module, event):
    return FloatChange(
        old_value=event.old_value,
        new_value=event.new_value
    )