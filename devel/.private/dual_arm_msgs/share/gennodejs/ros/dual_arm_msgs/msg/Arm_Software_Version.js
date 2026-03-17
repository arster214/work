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

class Arm_Software_Version {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Product_version = null;
      this.Plan_version = null;
    }
    else {
      if (initObj.hasOwnProperty('Product_version')) {
        this.Product_version = initObj.Product_version
      }
      else {
        this.Product_version = '';
      }
      if (initObj.hasOwnProperty('Plan_version')) {
        this.Plan_version = initObj.Plan_version
      }
      else {
        this.Plan_version = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Arm_Software_Version
    // Serialize message field [Product_version]
    bufferOffset = _serializer.string(obj.Product_version, buffer, bufferOffset);
    // Serialize message field [Plan_version]
    bufferOffset = _serializer.string(obj.Plan_version, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Arm_Software_Version
    let len;
    let data = new Arm_Software_Version(null);
    // Deserialize message field [Product_version]
    data.Product_version = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [Plan_version]
    data.Plan_version = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.Product_version);
    length += _getByteLength(object.Plan_version);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dual_arm_msgs/Arm_Software_Version';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '875ac4a1506e782c8b37a03d7f6b51c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Product_version
    string Plan_version
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Arm_Software_Version(null);
    if (msg.Product_version !== undefined) {
      resolved.Product_version = msg.Product_version;
    }
    else {
      resolved.Product_version = ''
    }

    if (msg.Plan_version !== undefined) {
      resolved.Plan_version = msg.Plan_version;
    }
    else {
      resolved.Plan_version = ''
    }

    return resolved;
    }
};

module.exports = Arm_Software_Version;
