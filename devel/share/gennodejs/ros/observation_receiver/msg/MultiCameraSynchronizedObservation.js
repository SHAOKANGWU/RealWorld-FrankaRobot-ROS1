// Auto-generated. Do not edit!

// (in-package observation_receiver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let franka_msgs = _finder('franka_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MultiCameraSynchronizedObservation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.camera_names = null;
      this.images = null;
      this.state = null;
      this.gripper_state = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('camera_names')) {
        this.camera_names = initObj.camera_names
      }
      else {
        this.camera_names = [];
      }
      if (initObj.hasOwnProperty('images')) {
        this.images = initObj.images
      }
      else {
        this.images = [];
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = new franka_msgs.msg.FrankaState();
      }
      if (initObj.hasOwnProperty('gripper_state')) {
        this.gripper_state = initObj.gripper_state
      }
      else {
        this.gripper_state = new sensor_msgs.msg.JointState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MultiCameraSynchronizedObservation
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [camera_names]
    bufferOffset = _arraySerializer.string(obj.camera_names, buffer, bufferOffset, null);
    // Serialize message field [images]
    // Serialize the length for message field [images]
    bufferOffset = _serializer.uint32(obj.images.length, buffer, bufferOffset);
    obj.images.forEach((val) => {
      bufferOffset = sensor_msgs.msg.Image.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [state]
    bufferOffset = franka_msgs.msg.FrankaState.serialize(obj.state, buffer, bufferOffset);
    // Serialize message field [gripper_state]
    bufferOffset = sensor_msgs.msg.JointState.serialize(obj.gripper_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MultiCameraSynchronizedObservation
    let len;
    let data = new MultiCameraSynchronizedObservation(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [camera_names]
    data.camera_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [images]
    // Deserialize array length for message field [images]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.images = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.images[i] = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [state]
    data.state = franka_msgs.msg.FrankaState.deserialize(buffer, bufferOffset);
    // Deserialize message field [gripper_state]
    data.gripper_state = sensor_msgs.msg.JointState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.camera_names.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.images.forEach((val) => {
      length += sensor_msgs.msg.Image.getMessageSize(val);
    });
    length += franka_msgs.msg.FrankaState.getMessageSize(object.state);
    length += sensor_msgs.msg.JointState.getMessageSize(object.gripper_state);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'observation_receiver/MultiCameraSynchronizedObservation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e93377bbf3fc506a140b041e450bd191';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Reference header from the first configured camera.
    std_msgs/Header header
    # Parallel arrays in configuration order: camera_names[i] identifies images[i].
    string[] camera_names
    sensor_msgs/Image[] images
    franka_msgs/FrankaState state
    sensor_msgs/JointState gripper_state
    
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
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
    ================================================================================
    MSG: franka_msgs/FrankaState
    std_msgs/Header header
    float64[6] cartesian_collision
    float64[6] cartesian_contact
    float64[7] q
    float64[7] q_d
    float64[7] dq
    float64[7] dq_d
    float64[7] ddq_d
    float64[7] theta
    float64[7] dtheta
    float64[7] tau_J
    float64[7] dtau_J
    float64[7] tau_J_d
    float64[6] K_F_ext_hat_K
    float64[2] elbow
    float64[2] elbow_d
    float64[2] elbow_c
    float64[2] delbow_c
    float64[2] ddelbow_c
    float64[7] joint_collision
    float64[7] joint_contact
    float64[6] O_F_ext_hat_K
    float64[6] O_dP_EE_d
    float64[3] O_ddP_O
    float64[6] O_dP_EE_c
    float64[6] O_ddP_EE_c
    float64[7] tau_ext_hat_filtered
    float64 m_ee
    float64[3] F_x_Cee
    float64[9] I_ee
    float64 m_load
    float64[3] F_x_Cload
    float64[9] I_load
    float64 m_total
    float64[3] F_x_Ctotal
    float64[9] I_total
    float64[16] O_T_EE
    float64[16] O_T_EE_d
    float64[16] O_T_EE_c
    float64[16] F_T_EE
    float64[16] F_T_NE
    float64[16] NE_T_EE
    float64[16] EE_T_K
    float64 time
    float64 control_command_success_rate
    uint8 ROBOT_MODE_OTHER=0
    uint8 ROBOT_MODE_IDLE=1
    uint8 ROBOT_MODE_MOVE=2
    uint8 ROBOT_MODE_GUIDING=3
    uint8 ROBOT_MODE_REFLEX=4
    uint8 ROBOT_MODE_USER_STOPPED=5
    uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6
    uint8 robot_mode
    franka_msgs/Errors current_errors
    franka_msgs/Errors last_motion_errors
    
    ================================================================================
    MSG: franka_msgs/Errors
    bool joint_position_limits_violation
    bool cartesian_position_limits_violation
    bool self_collision_avoidance_violation
    bool joint_velocity_violation
    bool cartesian_velocity_violation
    bool force_control_safety_violation
    bool joint_reflex
    bool cartesian_reflex
    bool max_goal_pose_deviation_violation
    bool max_path_pose_deviation_violation
    bool cartesian_velocity_profile_safety_violation
    bool joint_position_motion_generator_start_pose_invalid
    bool joint_motion_generator_position_limits_violation
    bool joint_motion_generator_velocity_limits_violation
    bool joint_motion_generator_velocity_discontinuity
    bool joint_motion_generator_acceleration_discontinuity
    bool cartesian_position_motion_generator_start_pose_invalid
    bool cartesian_motion_generator_elbow_limit_violation
    bool cartesian_motion_generator_velocity_limits_violation
    bool cartesian_motion_generator_velocity_discontinuity
    bool cartesian_motion_generator_acceleration_discontinuity
    bool cartesian_motion_generator_elbow_sign_inconsistent
    bool cartesian_motion_generator_start_elbow_invalid
    bool cartesian_motion_generator_joint_position_limits_violation
    bool cartesian_motion_generator_joint_velocity_limits_violation
    bool cartesian_motion_generator_joint_velocity_discontinuity
    bool cartesian_motion_generator_joint_acceleration_discontinuity
    bool cartesian_position_motion_generator_invalid_frame
    bool force_controller_desired_force_tolerance_violation
    bool controller_torque_discontinuity
    bool start_elbow_sign_inconsistent
    bool communication_constraints_violation
    bool power_limit_violation
    bool joint_p2p_insufficient_torque_for_planning
    bool tau_j_range_violation
    bool instability_detected
    bool joint_move_in_wrong_direction
    bool cartesian_spline_motion_generator_violation
    bool joint_via_motion_generator_planning_joint_limit_violation
    bool base_acceleration_initialization_timeout
    bool base_acceleration_invalid_reading
    
    ================================================================================
    MSG: sensor_msgs/JointState
    # This is a message that holds data to describe the state of a set of torque controlled joints. 
    #
    # The state of each joint (revolute or prismatic) is defined by:
    #  * the position of the joint (rad or m),
    #  * the velocity of the joint (rad/s or m/s) and 
    #  * the effort that is applied in the joint (Nm or N).
    #
    # Each joint is uniquely identified by its name
    # The header specifies the time at which the joint states were recorded. All the joint states
    # in one message have to be recorded at the same time.
    #
    # This message consists of a multiple arrays, one for each part of the joint state. 
    # The goal is to make each of the fields optional. When e.g. your joints have no
    # effort associated with them, you can leave the effort array empty. 
    #
    # All arrays in this message should have the same size, or be empty.
    # This is the only way to uniquely associate the joint name with the correct
    # states.
    
    
    Header header
    
    string[] name
    float64[] position
    float64[] velocity
    float64[] effort
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MultiCameraSynchronizedObservation(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.camera_names !== undefined) {
      resolved.camera_names = msg.camera_names;
    }
    else {
      resolved.camera_names = []
    }

    if (msg.images !== undefined) {
      resolved.images = new Array(msg.images.length);
      for (let i = 0; i < resolved.images.length; ++i) {
        resolved.images[i] = sensor_msgs.msg.Image.Resolve(msg.images[i]);
      }
    }
    else {
      resolved.images = []
    }

    if (msg.state !== undefined) {
      resolved.state = franka_msgs.msg.FrankaState.Resolve(msg.state)
    }
    else {
      resolved.state = new franka_msgs.msg.FrankaState()
    }

    if (msg.gripper_state !== undefined) {
      resolved.gripper_state = sensor_msgs.msg.JointState.Resolve(msg.gripper_state)
    }
    else {
      resolved.gripper_state = new sensor_msgs.msg.JointState()
    }

    return resolved;
    }
};

module.exports = MultiCameraSynchronizedObservation;
