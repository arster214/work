// Auto-generated. Do not edit!

// (in-package dual_arm_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CarteFdPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Pose_Euler = null;
    }
    else {
      if (initObj.hasOwnProperty('Pose_Euler')) {
        this.Pose_Euler = initObj.Pose_Euler
      }
      else {
        this.Pose_Euler = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CarteFdPose
    // Check that the constant length array field [Pose_Euler] has the right length
    if (obj.Pose_Euler.length !== 6) {
      throw new Error('Unable to serialize array field Pose_Euler - length must be 6')
    }
    // Serialize message field [Pose_Euler]
    bufferOffset = _arraySerializer.float32(obj.Pose_Euler, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CarteFdPose
    let len;
    let data = new CarteFdPose(null);
    // Deserialize message field [Pose_Euler]
    data.Pose_Euler = _arrayDeserializer.float32(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dual_arm_msgs/CarteFdPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '525ed60ead11361f867b19c646e09073';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[6] Pose_Euler
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CarteFdPose(null);
    if (msg.Pose_Euler !== undefined) {
      resolved.Pose_Euler = msg.Pose_Euler;
    }
    else {
      resolved.Pose_Euler = new Array(6).fill(0)
    }

    return resolved;
    }
};

module.exports = CarteFdPose;
