// Auto-generated. Do not edit!

// (in-package servo_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ServoAngle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.servo_id_1 = null;
      this.angle_1 = null;
      this.servo_id_2 = null;
      this.angle_2 = null;
    }
    else {
      if (initObj.hasOwnProperty('servo_id_1')) {
        this.servo_id_1 = initObj.servo_id_1
      }
      else {
        this.servo_id_1 = 0;
      }
      if (initObj.hasOwnProperty('angle_1')) {
        this.angle_1 = initObj.angle_1
      }
      else {
        this.angle_1 = 0;
      }
      if (initObj.hasOwnProperty('servo_id_2')) {
        this.servo_id_2 = initObj.servo_id_2
      }
      else {
        this.servo_id_2 = 0;
      }
      if (initObj.hasOwnProperty('angle_2')) {
        this.angle_2 = initObj.angle_2
      }
      else {
        this.angle_2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ServoAngle
    // Serialize message field [servo_id_1]
    bufferOffset = _serializer.uint16(obj.servo_id_1, buffer, bufferOffset);
    // Serialize message field [angle_1]
    bufferOffset = _serializer.uint16(obj.angle_1, buffer, bufferOffset);
    // Serialize message field [servo_id_2]
    bufferOffset = _serializer.uint16(obj.servo_id_2, buffer, bufferOffset);
    // Serialize message field [angle_2]
    bufferOffset = _serializer.uint16(obj.angle_2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ServoAngle
    let len;
    let data = new ServoAngle(null);
    // Deserialize message field [servo_id_1]
    data.servo_id_1 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [angle_1]
    data.angle_1 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [servo_id_2]
    data.servo_id_2 = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [angle_2]
    data.angle_2 = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'servo_ros/ServoAngle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9a8103b4cee119f9447f2067818f6250';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #舵机返回角度
    uint16 servo_id_1	#舵机ID
    uint16 angle_1	#角度位置0~1000
    uint16 servo_id_2	#舵机ID
    uint16 angle_2	#角度位置0~1000
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ServoAngle(null);
    if (msg.servo_id_1 !== undefined) {
      resolved.servo_id_1 = msg.servo_id_1;
    }
    else {
      resolved.servo_id_1 = 0
    }

    if (msg.angle_1 !== undefined) {
      resolved.angle_1 = msg.angle_1;
    }
    else {
      resolved.angle_1 = 0
    }

    if (msg.servo_id_2 !== undefined) {
      resolved.servo_id_2 = msg.servo_id_2;
    }
    else {
      resolved.servo_id_2 = 0
    }

    if (msg.angle_2 !== undefined) {
      resolved.angle_2 = msg.angle_2;
    }
    else {
      resolved.angle_2 = 0
    }

    return resolved;
    }
};

module.exports = ServoAngle;
