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

class ESKFStd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.delta_pos_x_std = null;
      this.delta_pos_y_std = null;
      this.delta_pos_z_std = null;
      this.delta_vel_x_std = null;
      this.delta_vel_y_std = null;
      this.delta_vel_z_std = null;
      this.delta_ori_x_std = null;
      this.delta_ori_y_std = null;
      this.delta_ori_z_std = null;
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
      if (initObj.hasOwnProperty('delta_pos_x_std')) {
        this.delta_pos_x_std = initObj.delta_pos_x_std
      }
      else {
        this.delta_pos_x_std = 0.0;
      }
      if (initObj.hasOwnProperty('delta_pos_y_std')) {
        this.delta_pos_y_std = initObj.delta_pos_y_std
      }
      else {
        this.delta_pos_y_std = 0.0;
      }
      if (initObj.hasOwnProperty('delta_pos_z_std')) {
        this.delta_pos_z_std = initObj.delta_pos_z_std
      }
      else {
        this.delta_pos_z_std = 0.0;
      }
      if (initObj.hasOwnProperty('delta_vel_x_std')) {
        this.delta_vel_x_std = initObj.delta_vel_x_std
      }
      else {
        this.delta_vel_x_std = 0.0;
      }
      if (initObj.hasOwnProperty('delta_vel_y_std')) {
        this.delta_vel_y_std = initObj.delta_vel_y_std
      }
      else {
        this.delta_vel_y_std = 0.0;
      }
      if (initObj.hasOwnProperty('delta_vel_z_std')) {
        this.delta_vel_z_std = initObj.delta_vel_z_std
      }
      else {
        this.delta_vel_z_std = 0.0;
      }
      if (initObj.hasOwnProperty('delta_ori_x_std')) {
        this.delta_ori_x_std = initObj.delta_ori_x_std
      }
      else {
        this.delta_ori_x_std = 0.0;
      }
      if (initObj.hasOwnProperty('delta_ori_y_std')) {
        this.delta_ori_y_std = initObj.delta_ori_y_std
      }
      else {
        this.delta_ori_y_std = 0.0;
      }
      if (initObj.hasOwnProperty('delta_ori_z_std')) {
        this.delta_ori_z_std = initObj.delta_ori_z_std
      }
      else {
        this.delta_ori_z_std = 0.0;
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
    // Serializes a message object of type ESKFStd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [delta_pos_x_std]
    bufferOffset = _serializer.float64(obj.delta_pos_x_std, buffer, bufferOffset);
    // Serialize message field [delta_pos_y_std]
    bufferOffset = _serializer.float64(obj.delta_pos_y_std, buffer, bufferOffset);
    // Serialize message field [delta_pos_z_std]
    bufferOffset = _serializer.float64(obj.delta_pos_z_std, buffer, bufferOffset);
    // Serialize message field [delta_vel_x_std]
    bufferOffset = _serializer.float64(obj.delta_vel_x_std, buffer, bufferOffset);
    // Serialize message field [delta_vel_y_std]
    bufferOffset = _serializer.float64(obj.delta_vel_y_std, buffer, bufferOffset);
    // Serialize message field [delta_vel_z_std]
    bufferOffset = _serializer.float64(obj.delta_vel_z_std, buffer, bufferOffset);
    // Serialize message field [delta_ori_x_std]
    bufferOffset = _serializer.float64(obj.delta_ori_x_std, buffer, bufferOffset);
    // Serialize message field [delta_ori_y_std]
    bufferOffset = _serializer.float64(obj.delta_ori_y_std, buffer, bufferOffset);
    // Serialize message field [delta_ori_z_std]
    bufferOffset = _serializer.float64(obj.delta_ori_z_std, buffer, bufferOffset);
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
    //deserializes a message object of type ESKFStd
    let len;
    let data = new ESKFStd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [delta_pos_x_std]
    data.delta_pos_x_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_pos_y_std]
    data.delta_pos_y_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_pos_z_std]
    data.delta_pos_z_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_vel_x_std]
    data.delta_vel_x_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_vel_y_std]
    data.delta_vel_y_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_vel_z_std]
    data.delta_vel_z_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_ori_x_std]
    data.delta_ori_x_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_ori_y_std]
    data.delta_ori_y_std = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_ori_z_std]
    data.delta_ori_z_std = _deserializer.float64(buffer, bufferOffset);
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
    return length + 120;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidar_localization/ESKFStd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ab13091af10d5ae8e76adaf8e34014b3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # time of ESKF estimation:
    Header header
    
    # a. position:
    float64 delta_pos_x_std
    float64 delta_pos_y_std
    float64 delta_pos_z_std
    
    # b. velocity:
    float64 delta_vel_x_std
    float64 delta_vel_y_std
    float64 delta_vel_z_std
    
    # c. orientation:
    float64 delta_ori_x_std
    float64 delta_ori_y_std
    float64 delta_ori_z_std
    
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
    const resolved = new ESKFStd(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.delta_pos_x_std !== undefined) {
      resolved.delta_pos_x_std = msg.delta_pos_x_std;
    }
    else {
      resolved.delta_pos_x_std = 0.0
    }

    if (msg.delta_pos_y_std !== undefined) {
      resolved.delta_pos_y_std = msg.delta_pos_y_std;
    }
    else {
      resolved.delta_pos_y_std = 0.0
    }

    if (msg.delta_pos_z_std !== undefined) {
      resolved.delta_pos_z_std = msg.delta_pos_z_std;
    }
    else {
      resolved.delta_pos_z_std = 0.0
    }

    if (msg.delta_vel_x_std !== undefined) {
      resolved.delta_vel_x_std = msg.delta_vel_x_std;
    }
    else {
      resolved.delta_vel_x_std = 0.0
    }

    if (msg.delta_vel_y_std !== undefined) {
      resolved.delta_vel_y_std = msg.delta_vel_y_std;
    }
    else {
      resolved.delta_vel_y_std = 0.0
    }

    if (msg.delta_vel_z_std !== undefined) {
      resolved.delta_vel_z_std = msg.delta_vel_z_std;
    }
    else {
      resolved.delta_vel_z_std = 0.0
    }

    if (msg.delta_ori_x_std !== undefined) {
      resolved.delta_ori_x_std = msg.delta_ori_x_std;
    }
    else {
      resolved.delta_ori_x_std = 0.0
    }

    if (msg.delta_ori_y_std !== undefined) {
      resolved.delta_ori_y_std = msg.delta_ori_y_std;
    }
    else {
      resolved.delta_ori_y_std = 0.0
    }

    if (msg.delta_ori_z_std !== undefined) {
      resolved.delta_ori_z_std = msg.delta_ori_z_std;
    }
    else {
      resolved.delta_ori_z_std = 0.0
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

module.exports = ESKFStd;
