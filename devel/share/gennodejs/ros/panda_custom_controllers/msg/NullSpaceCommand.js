// Auto-generated. Do not edit!

// (in-package panda_custom_controllers.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class NullSpaceCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pose = null;
      this.joint_positions = null;
      this.gripper_grasp = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('joint_positions')) {
        this.joint_positions = initObj.joint_positions
      }
      else {
        this.joint_positions = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('gripper_grasp')) {
        this.gripper_grasp = initObj.gripper_grasp
      }
      else {
        this.gripper_grasp = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NullSpaceCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Check that the constant length array field [joint_positions] has the right length
    if (obj.joint_positions.length !== 7) {
      throw new Error('Unable to serialize array field joint_positions - length must be 7')
    }
    // Serialize message field [joint_positions]
    bufferOffset = _arraySerializer.float64(obj.joint_positions, buffer, bufferOffset, 7);
    // Serialize message field [gripper_grasp]
    bufferOffset = _serializer.bool(obj.gripper_grasp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NullSpaceCommand
    let len;
    let data = new NullSpaceCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_positions]
    data.joint_positions = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [gripper_grasp]
    data.gripper_grasp = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 113;
  }

  static datatype() {
    // Returns string type for a message object
    return 'panda_custom_controllers/NullSpaceCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac7f965b4f3f525107f6071ff3534376';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    geometry_msgs/Pose pose
    float64[7] joint_positions
    bool gripper_grasp
    
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
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NullSpaceCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.joint_positions !== undefined) {
      resolved.joint_positions = msg.joint_positions;
    }
    else {
      resolved.joint_positions = new Array(7).fill(0)
    }

    if (msg.gripper_grasp !== undefined) {
      resolved.gripper_grasp = msg.gripper_grasp;
    }
    else {
      resolved.gripper_grasp = false
    }

    return resolved;
    }
};

module.exports = NullSpaceCommand;
