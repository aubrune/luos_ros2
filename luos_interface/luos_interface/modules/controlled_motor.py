from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3
from .serializers import serializeVector3, serializeFloat32, serializeBool
from .deserializers import deserializeVector3, deserializeFloat32, deserializeBool
from .generic import LuosGenericPublisher


class LuosControlledMotorPublisher(LuosGenericPublisher):
    def __init__(self, node, module, rate):
        variables = {
            "positionPid": {"type": Vector3,
                      "serialize": serializeVector3, "deserialize": deserializeVector3,
                      "read": True, "write": True},
            "speedPid": {"type": Vector3,
                     "serialize": serializeVector3, "deserialize": deserializeVector3,
                     "read": True, "write": True},
            "encoder_res": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "reduction": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "wheel_size": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "compliant": {"type": Bool, "read": True, "write": True,
                          "serialize": serializeBool, "deserialize": deserializeBool},
            "power_mode": {"type": Bool, "read": True, "write": True,
                          "serialize": serializeBool, "deserialize": deserializeBool},
            "rot_position_mode": {"type": Bool, "read": True, "write": True,
                          "serialize": serializeBool, "deserialize": deserializeBool},
            "rot_speed_mode": {"type": Bool, "read": True, "write": True,
                          "serialize": serializeBool, "deserialize": deserializeBool},
            "trans_position_mode": {"type": Bool, "read": True, "write": True,
                          "serialize": serializeBool, "deserialize": deserializeBool},
            "trans_speed_mode": {"type": Bool, "read": True, "write": True,
                          "serialize": serializeBool, "deserialize": deserializeBool},
            "rot_position": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeBool},
            "rot_speed": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeBool},
            "trans_position": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeBool},
            "trans_speed": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeBool},
            "current": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeBool},
            "power_ratio": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "target_rot_position": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "target_rot_speed": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "target_trans_position": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeFloat32},
            "target_trans_speed": {"type": Float32, "read": True, "write": True,
                          "serialize": serializeFloat32, "deserialize": deserializeFloat32},
        }
        events = {}
        aggregates = {}
        super(LuosControlledMotorPublisher, self).__init__(node, module, rate, variables, events, aggregates)
