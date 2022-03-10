// Auto-generated. Do not edit!

// (in-package lidar_localization.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class EKFStd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pos_x_std = null;
      this.pos_y_std = null;
      this.pos_z_std = null;
      this.vel_x_std = null;
      this.vel_y_std = null;
      this.vel_z_std = null;
      this.ori_w_std = null;
      this.ori_x_std = null;
      this.ori_y_std = null;
      this.ori_z_std = null;
      this.gyro_bias_x_std = null;
      this.gyro_bias_y_std = null;
      this.gyro_bias_z_std = null;
      this.accel_bias_x_std = null;
      this.accel_bias_y_std = null;
      this.accel_bias_z_std = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pos_x_std')) {
        this.pos_x_std = initObj.pos_x_std
      }
      else {
        this.pos_x_std = 0.0;
      }
      if (initObj.hasOwnProperty('pos_y_std')) {
        this.pos_y_std = initObj.pos_y_std
      }
      else {
        this.pos_y_std = 0.0;
      }
      if (initObj.hasOwnProperty('pos_z_std')) {
        this.pos_z_std = initObj.pos_z_std
      }
      else {
        this.pos_z_std = 0.0;
      }
      if (initObj.hasOwnProperty('vel_x_std')) {
        this.vel_x_std = initObj.vel_x_std
      }
      else {
        this.vel_x_std = 0.0;
      }
      if (initObj.hasOwnProperty('vel_y_std')) {
        this.vel_y_std = initObj.vel_y_std
      }
      else {
        this.vel_y_std = 0.0;
      }
      if (initObj.hasOwnProperty('vel_z_std')) {
        this.vel_z_std = initObj.vel_z_std
      }
      else {
        this.vel_z_std = 0.0;
      }
      if (initObj.hasOwnProperty('ori_w_std')) {
        this.ori_w_std = initObj.ori_w_std
      }
      else {
        this.ori_w_std = 0.0;
      }
      if (initObj.hasOwnProperty('ori_x_std')) {
        this.ori_x_std = initObj.ori_x_std
      }
      else {
        this.ori_x_std = 0.0;
      }
      if (initObj.hasOwnProperty('ori_y_std')) {
        this.ori_y_std = initObj.ori_y_std
      }
      else {
        this.ori_y_std = 0.0;
      }
      if (initObj.hasOwnProperty('ori_z_std')) {
        this.ori_z_std = initObj.ori_z_std
      }
      else {
        this.ori_z_std = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_bias_x_std')) {
        this.gyro_bias_x_std = initObj.gyro_bias_x_std
      }
      else {
        this.gyro_bias_x_std = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_bias_y_std')) {
        this.gyro_bias_y_std = initObj.gyro_bias_y_std
      }
      else {
        this.gyro_bias_y_std = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_bias_z_std')) {
        this.gyro_bias_z_std = initObj.gyro_bias_z_std
      }
      else {
        this.gyro_bias_z_std = 0.0;
      }
      if (initObj.hasOwnProperty('accel_bias_x_std')) {
        this.accel_bias_x_std = initObj.accel_bias_x_std
      }
      else {
        this.accel_bias_x_std = 0.0;
      }
      if (initObj.hasOwnProperty('accel_bias_y_std')) {
        this.accel_bias_y_std = initObj.accel_bias_y_std
      }
      else {
        this.accel_bias_y_std = 0.0;
      }
      if (initObj.hasOwnProperty('accel_bias_z_std')) {
        this.accel_bias_z_std = initObj.accel_bias_z_std
      }
      else {
        this.accel_bias_z_std = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EKFStd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pos_x_std]
    bufferOffset = _serializer.float64(obj.pos_x_std, buffer, bufferOffset);
    // Serialize message field [pos_y_std]
    bufferOffset = _serializer.float64(obj.pos_y_std, buffer, bufferOffset);
    // Serialize message field [pos_z_std]
    bufferOffset = _serializer.float64(obj.pos_z_std, buffer, bufferOffset);
    // Serialize message field [vel_x_std]
    bufferOffset = _serializer.float64(obj.vel_x_std, buffer, bufferOffset);
    // Serialize message field [vel_y_std]
    bufferOffset = _serializer.float64(obj.vel_y_std, buffer, bufferOffset);
    // Serialize message field [vel_z_std]
    bufferOffset = _serializer.float64(obj.vel_z_std, buffer, bufferOffset);
    // Serialize message field [ori_w_std]
    bufferOffset = _serializer.float64(obj.ori_w_std, buffer, bufferOffset);
    // Serialize message field [ori_x_std]
    bufferOffset = _serializer.float64(obj.ori_x_std, buffer, bufferOffset);
    // Serialize message field [ori_y_std]
    bufferOffset = _serializer.float64(obj.ori_y_std, buffer, bufferOffset);
    // Serialize message field [ori_z_std]
    bufferOffset = _serializer.float64(obj.ori_z_std, buffer, bufferOffset);
    // Serialize message field [gyro_bias_x_std]
    bufferOffset = _serializer.float64(obj.gyro_bias_x_std, buffer, bufferOffset);
    // Serialize message field [gyro_bias_y_std]
    bufferOffset = _serializer.float64(obj.gyro_bias_y_std, buffer, bufferOffset);
    // Serialize message field [gyro_bias_z_std]
    bufferOffset = _serializer.float64(obj.gyro_bias_z_std, buffer, bufferOffset);
    // Serialize message field [accel_bias_x_std]
    bufferOffset = _serializer.float64(obj.accel_bias_x_std, buffer, bufferOffset);
    // Serialize message field [accel_bias_y_std]
    bufferOffset = _serializer.float64(obj.accel_bias_y_std, buffer, bufferOffset);
    // Serialize message field [accel_bias_z_std]
    bufferOffset = _serializer.float64(obj.accel_bias_z_std, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EKFStd
    let len;
    let data = new EKFStd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pos_x_std]
    data.pos_x_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pos_y_std]
    data.pos_y_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pos_z_std]
    data.pos_z_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_x_std]
    data.vel_x_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_y_std]
    data.vel_y_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_z_std]
    data.vel_z_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ori_w_std]
    data.ori_w_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ori_x_std]
    data.ori_x_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ori_y_std]
    data.ori_y_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ori_z_std]
    data.ori_z_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gyro_bias_x_std]
    data.gyro_bias_x_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gyro_bias_y_std]
    data.gyro_bias_y_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gyro_bias_z_std]
    data.gyro_bias_z_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accel_bias_x_std]
    data.accel_bias_x_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accel_bias_y_std]
    data.accel_bias_y_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [accel_bias_z_std]
    data.accel_bias_z_std = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 128;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidar_localization/EKFStd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2e9c1dfcaa6649033a40105a26a0f620';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # time of ESKF estimation:
    Header header
    
    # a. position:
    float64 pos_x_std
    float64 pos_y_std
    float64 pos_z_std
    
    # b. velocity:
    float64 vel_x_std
    float64 vel_y_std
    float64 vel_z_std
    
    # c. orientation:
    float64 ori_w_std
    float64 ori_x_std
    float64 ori_y_std
    float64 ori_z_std
    
    # d. gyro. bias:
    float64 gyro_bias_x_std
    float64 gyro_bias_y_std
    float64 gyro_bias_z_std
    
    # e. accel. bias:
    float64 accel_bias_x_std
    float64 accel_bias_y_std
    float64 accel_bias_z_std
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EKFStd(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pos_x_std !== undefined) {
      resolved.pos_x_std = msg.pos_x_std;
    }
    else {
      resolved.pos_x_std = 0.0
    }

    if (msg.pos_y_std !== undefined) {
      resolved.pos_y_std = msg.pos_y_std;
    }
    else {
      resolved.pos_y_std = 0.0
    }

    if (msg.pos_z_std !== undefined) {
      resolved.pos_z_std = msg.pos_z_std;
    }
    else {
      resolved.pos_z_std = 0.0
    }

    if (msg.vel_x_std !== undefined) {
      resolved.vel_x_std = msg.vel_x_std;
    }
    else {
      resolved.vel_x_std = 0.0
    }

    if (msg.vel_y_std !== undefined) {
      resolved.vel_y_std = msg.vel_y_std;
    }
    else {
      resolved.vel_y_std = 0.0
    }

    if (msg.vel_z_std !== undefined) {
      resolved.vel_z_std = msg.vel_z_std;
    }
    else {
      resolved.vel_z_std = 0.0
    }

    if (msg.ori_w_std !== undefined) {
      resolved.ori_w_std = msg.ori_w_std;
    }
    else {
      resolved.ori_w_std = 0.0
    }

    if (msg.ori_x_std !== undefined) {
      resolved.ori_x_std = msg.ori_x_std;
    }
    else {
      resolved.ori_x_std = 0.0
    }

    if (msg.ori_y_std !== undefined) {
      resolved.ori_y_std = msg.ori_y_std;
    }
    else {
      resolved.ori_y_std = 0.0
    }

    if (msg.ori_z_std !== undefined) {
      resolved.ori_z_std = msg.ori_z_std;
    }
    else {
      resolved.ori_z_std = 0.0
    }

    if (msg.gyro_bias_x_std !== undefined) {
      resolved.gyro_bias_x_std = msg.gyro_bias_x_std;
    }
    else {
      resolved.gyro_bias_x_std = 0.0
    }

    if (msg.gyro_bias_y_std !== undefined) {
      resolved.gyro_bias_y_std = msg.gyro_bias_y_std;
    }
    else {
      resolved.gyro_bias_y_std = 0.0
    }

    if (msg.gyro_bias_z_std !== undefined) {
      resolved.gyro_bias_z_std = msg.gyro_bias_z_std;
    }
    else {
      resolved.gyro_bias_z_std = 0.0
    }

    if (msg.accel_bias_x_std !== undefined) {
      resolved.accel_bias_x_std = msg.accel_bias_x_std;
    }
    else {
      resolved.accel_bias_x_std = 0.0
    }

    if (msg.accel_bias_y_std !== undefined) {
      resolved.accel_bias_y_std = msg.accel_bias_y_std;
    }
    else {
      resolved.accel_bias_y_std = 0.0
    }

    if (msg.accel_bias_z_std !== undefined) {
      resolved.accel_bias_z_std = msg.accel_bias_z_std;
    }
    else {
      resolved.accel_bias_z_std = 0.0
    }

    return resolved;
    }
};

module.exports = EKFStd;
