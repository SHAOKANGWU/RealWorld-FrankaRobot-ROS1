// Auto-generated. Do not edit!

// (in-package panda_spacemouse_teleop.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GripperCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.command = null;
      this.width = null;
      this.speed = null;
      this.force = null;
      this.epsilon_inner = null;
      this.epsilon_outer = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = 0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('force')) {
        this.force = initObj.force
      }
      else {
        this.force = 0.0;
      }
      if (initObj.hasOwnProperty('epsilon_inner')) {
        this.epsilon_inner = initObj.epsilon_inner
      }
      else {
        this.epsilon_inner = 0.0;
      }
      if (initObj.hasOwnProperty('epsilon_outer')) {
        this.epsilon_outer = initObj.epsilon_outer
      }
      else {
        this.epsilon_outer = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = _serializer.uint8(obj.command, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.float64(obj.width, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float64(obj.speed, buffer, bufferOffset);
    // Serialize message field [force]
    bufferOffset = _serializer.float64(obj.force, buffer, bufferOffset);
    // Serialize message field [epsilon_inner]
    bufferOffset = _serializer.float64(obj.epsilon_inner, buffer, bufferOffset);
    // Serialize message field [epsilon_outer]
    bufferOffset = _serializer.float64(obj.epsilon_outer, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperCommand
    let len;
    let data = new GripperCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [force]
    data.force = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [epsilon_inner]
    data.epsilon_inner = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [epsilon_outer]
    data.epsilon_outer = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 41;
  }

  static datatype() {
    // Returns string type for a message object
    return 'panda_spacemouse_teleop/GripperCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '409a402d2a8c08a8d67a9748adb6125d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 OPEN=0
    uint8 GRASP=1
    uint8 command
    
    float64 width
    float64 speed
    float64 force
    float64 epsilon_inner
    float64 epsilon_outer
    
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
    const resolved = new GripperCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = 0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.force !== undefined) {
      resolved.force = msg.force;
    }
    else {
      resolved.force = 0.0
    }

    if (msg.epsilon_inner !== undefined) {
      resolved.epsilon_inner = msg.epsilon_inner;
    }
    else {
      resolved.epsilon_inner = 0.0
    }

    if (msg.epsilon_outer !== undefined) {
      resolved.epsilon_outer = msg.epsilon_outer;
    }
    else {
      resolved.epsilon_outer = 0.0
    }

    return resolved;
    }
};

// Constants for message
GripperCommand.Constants = {
  OPEN: 0,
  GRASP: 1,
}

module.exports = GripperCommand;
