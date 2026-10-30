; Auto-generated. Do not edit!


(cl:in-package observation_receiver-msg)


;//! \htmlinclude MultiCameraSynchronizedObservation.msg.html

(cl:defclass <MultiCameraSynchronizedObservation> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (camera_names
    :reader camera_names
    :initarg :camera_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (images
    :reader images
    :initarg :images
    :type (cl:vector sensor_msgs-msg:Image)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:Image :initial-element (cl:make-instance 'sensor_msgs-msg:Image)))
   (state
    :reader state
    :initarg :state
    :type franka_msgs-msg:FrankaState
    :initform (cl:make-instance 'franka_msgs-msg:FrankaState))
   (gripper_state
    :reader gripper_state
    :initarg :gripper_state
    :type sensor_msgs-msg:JointState
    :initform (cl:make-instance 'sensor_msgs-msg:JointState)))
)

(cl:defclass MultiCameraSynchronizedObservation (<MultiCameraSynchronizedObservation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MultiCameraSynchronizedObservation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MultiCameraSynchronizedObservation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name observation_receiver-msg:<MultiCameraSynchronizedObservation> is deprecated: use observation_receiver-msg:MultiCameraSynchronizedObservation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MultiCameraSynchronizedObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader observation_receiver-msg:header-val is deprecated.  Use observation_receiver-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'camera_names-val :lambda-list '(m))
(cl:defmethod camera_names-val ((m <MultiCameraSynchronizedObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader observation_receiver-msg:camera_names-val is deprecated.  Use observation_receiver-msg:camera_names instead.")
  (camera_names m))

(cl:ensure-generic-function 'images-val :lambda-list '(m))
(cl:defmethod images-val ((m <MultiCameraSynchronizedObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader observation_receiver-msg:images-val is deprecated.  Use observation_receiver-msg:images instead.")
  (images m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <MultiCameraSynchronizedObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader observation_receiver-msg:state-val is deprecated.  Use observation_receiver-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'gripper_state-val :lambda-list '(m))
(cl:defmethod gripper_state-val ((m <MultiCameraSynchronizedObservation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader observation_receiver-msg:gripper_state-val is deprecated.  Use observation_receiver-msg:gripper_state instead.")
  (gripper_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MultiCameraSynchronizedObservation>) ostream)
  "Serializes a message object of type '<MultiCameraSynchronizedObservation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'camera_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'camera_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'images))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'images))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gripper_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MultiCameraSynchronizedObservation>) istream)
  "Deserializes a message object of type '<MultiCameraSynchronizedObservation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'camera_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'camera_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'images) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'images)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:Image))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gripper_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MultiCameraSynchronizedObservation>)))
  "Returns string type for a message object of type '<MultiCameraSynchronizedObservation>"
  "observation_receiver/MultiCameraSynchronizedObservation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MultiCameraSynchronizedObservation)))
  "Returns string type for a message object of type 'MultiCameraSynchronizedObservation"
  "observation_receiver/MultiCameraSynchronizedObservation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MultiCameraSynchronizedObservation>)))
  "Returns md5sum for a message object of type '<MultiCameraSynchronizedObservation>"
  "e93377bbf3fc506a140b041e450bd191")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MultiCameraSynchronizedObservation)))
  "Returns md5sum for a message object of type 'MultiCameraSynchronizedObservation"
  "e93377bbf3fc506a140b041e450bd191")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MultiCameraSynchronizedObservation>)))
  "Returns full string definition for message of type '<MultiCameraSynchronizedObservation>"
  (cl:format cl:nil "# Reference header from the first configured camera.~%std_msgs/Header header~%# Parallel arrays in configuration order: camera_names[i] identifies images[i].~%string[] camera_names~%sensor_msgs/Image[] images~%franka_msgs/FrankaState state~%sensor_msgs/JointState gripper_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: franka_msgs/FrankaState~%std_msgs/Header header~%float64[6] cartesian_collision~%float64[6] cartesian_contact~%float64[7] q~%float64[7] q_d~%float64[7] dq~%float64[7] dq_d~%float64[7] ddq_d~%float64[7] theta~%float64[7] dtheta~%float64[7] tau_J~%float64[7] dtau_J~%float64[7] tau_J_d~%float64[6] K_F_ext_hat_K~%float64[2] elbow~%float64[2] elbow_d~%float64[2] elbow_c~%float64[2] delbow_c~%float64[2] ddelbow_c~%float64[7] joint_collision~%float64[7] joint_contact~%float64[6] O_F_ext_hat_K~%float64[6] O_dP_EE_d~%float64[3] O_ddP_O~%float64[6] O_dP_EE_c~%float64[6] O_ddP_EE_c~%float64[7] tau_ext_hat_filtered~%float64 m_ee~%float64[3] F_x_Cee~%float64[9] I_ee~%float64 m_load~%float64[3] F_x_Cload~%float64[9] I_load~%float64 m_total~%float64[3] F_x_Ctotal~%float64[9] I_total~%float64[16] O_T_EE~%float64[16] O_T_EE_d~%float64[16] O_T_EE_c~%float64[16] F_T_EE~%float64[16] F_T_NE~%float64[16] NE_T_EE~%float64[16] EE_T_K~%float64 time~%float64 control_command_success_rate~%uint8 ROBOT_MODE_OTHER=0~%uint8 ROBOT_MODE_IDLE=1~%uint8 ROBOT_MODE_MOVE=2~%uint8 ROBOT_MODE_GUIDING=3~%uint8 ROBOT_MODE_REFLEX=4~%uint8 ROBOT_MODE_USER_STOPPED=5~%uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6~%uint8 robot_mode~%franka_msgs/Errors current_errors~%franka_msgs/Errors last_motion_errors~%~%================================================================================~%MSG: franka_msgs/Errors~%bool joint_position_limits_violation~%bool cartesian_position_limits_violation~%bool self_collision_avoidance_violation~%bool joint_velocity_violation~%bool cartesian_velocity_violation~%bool force_control_safety_violation~%bool joint_reflex~%bool cartesian_reflex~%bool max_goal_pose_deviation_violation~%bool max_path_pose_deviation_violation~%bool cartesian_velocity_profile_safety_violation~%bool joint_position_motion_generator_start_pose_invalid~%bool joint_motion_generator_position_limits_violation~%bool joint_motion_generator_velocity_limits_violation~%bool joint_motion_generator_velocity_discontinuity~%bool joint_motion_generator_acceleration_discontinuity~%bool cartesian_position_motion_generator_start_pose_invalid~%bool cartesian_motion_generator_elbow_limit_violation~%bool cartesian_motion_generator_velocity_limits_violation~%bool cartesian_motion_generator_velocity_discontinuity~%bool cartesian_motion_generator_acceleration_discontinuity~%bool cartesian_motion_generator_elbow_sign_inconsistent~%bool cartesian_motion_generator_start_elbow_invalid~%bool cartesian_motion_generator_joint_position_limits_violation~%bool cartesian_motion_generator_joint_velocity_limits_violation~%bool cartesian_motion_generator_joint_velocity_discontinuity~%bool cartesian_motion_generator_joint_acceleration_discontinuity~%bool cartesian_position_motion_generator_invalid_frame~%bool force_controller_desired_force_tolerance_violation~%bool controller_torque_discontinuity~%bool start_elbow_sign_inconsistent~%bool communication_constraints_violation~%bool power_limit_violation~%bool joint_p2p_insufficient_torque_for_planning~%bool tau_j_range_violation~%bool instability_detected~%bool joint_move_in_wrong_direction~%bool cartesian_spline_motion_generator_violation~%bool joint_via_motion_generator_planning_joint_limit_violation~%bool base_acceleration_initialization_timeout~%bool base_acceleration_invalid_reading~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MultiCameraSynchronizedObservation)))
  "Returns full string definition for message of type 'MultiCameraSynchronizedObservation"
  (cl:format cl:nil "# Reference header from the first configured camera.~%std_msgs/Header header~%# Parallel arrays in configuration order: camera_names[i] identifies images[i].~%string[] camera_names~%sensor_msgs/Image[] images~%franka_msgs/FrankaState state~%sensor_msgs/JointState gripper_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: franka_msgs/FrankaState~%std_msgs/Header header~%float64[6] cartesian_collision~%float64[6] cartesian_contact~%float64[7] q~%float64[7] q_d~%float64[7] dq~%float64[7] dq_d~%float64[7] ddq_d~%float64[7] theta~%float64[7] dtheta~%float64[7] tau_J~%float64[7] dtau_J~%float64[7] tau_J_d~%float64[6] K_F_ext_hat_K~%float64[2] elbow~%float64[2] elbow_d~%float64[2] elbow_c~%float64[2] delbow_c~%float64[2] ddelbow_c~%float64[7] joint_collision~%float64[7] joint_contact~%float64[6] O_F_ext_hat_K~%float64[6] O_dP_EE_d~%float64[3] O_ddP_O~%float64[6] O_dP_EE_c~%float64[6] O_ddP_EE_c~%float64[7] tau_ext_hat_filtered~%float64 m_ee~%float64[3] F_x_Cee~%float64[9] I_ee~%float64 m_load~%float64[3] F_x_Cload~%float64[9] I_load~%float64 m_total~%float64[3] F_x_Ctotal~%float64[9] I_total~%float64[16] O_T_EE~%float64[16] O_T_EE_d~%float64[16] O_T_EE_c~%float64[16] F_T_EE~%float64[16] F_T_NE~%float64[16] NE_T_EE~%float64[16] EE_T_K~%float64 time~%float64 control_command_success_rate~%uint8 ROBOT_MODE_OTHER=0~%uint8 ROBOT_MODE_IDLE=1~%uint8 ROBOT_MODE_MOVE=2~%uint8 ROBOT_MODE_GUIDING=3~%uint8 ROBOT_MODE_REFLEX=4~%uint8 ROBOT_MODE_USER_STOPPED=5~%uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6~%uint8 robot_mode~%franka_msgs/Errors current_errors~%franka_msgs/Errors last_motion_errors~%~%================================================================================~%MSG: franka_msgs/Errors~%bool joint_position_limits_violation~%bool cartesian_position_limits_violation~%bool self_collision_avoidance_violation~%bool joint_velocity_violation~%bool cartesian_velocity_violation~%bool force_control_safety_violation~%bool joint_reflex~%bool cartesian_reflex~%bool max_goal_pose_deviation_violation~%bool max_path_pose_deviation_violation~%bool cartesian_velocity_profile_safety_violation~%bool joint_position_motion_generator_start_pose_invalid~%bool joint_motion_generator_position_limits_violation~%bool joint_motion_generator_velocity_limits_violation~%bool joint_motion_generator_velocity_discontinuity~%bool joint_motion_generator_acceleration_discontinuity~%bool cartesian_position_motion_generator_start_pose_invalid~%bool cartesian_motion_generator_elbow_limit_violation~%bool cartesian_motion_generator_velocity_limits_violation~%bool cartesian_motion_generator_velocity_discontinuity~%bool cartesian_motion_generator_acceleration_discontinuity~%bool cartesian_motion_generator_elbow_sign_inconsistent~%bool cartesian_motion_generator_start_elbow_invalid~%bool cartesian_motion_generator_joint_position_limits_violation~%bool cartesian_motion_generator_joint_velocity_limits_violation~%bool cartesian_motion_generator_joint_velocity_discontinuity~%bool cartesian_motion_generator_joint_acceleration_discontinuity~%bool cartesian_position_motion_generator_invalid_frame~%bool force_controller_desired_force_tolerance_violation~%bool controller_torque_discontinuity~%bool start_elbow_sign_inconsistent~%bool communication_constraints_violation~%bool power_limit_violation~%bool joint_p2p_insufficient_torque_for_planning~%bool tau_j_range_violation~%bool instability_detected~%bool joint_move_in_wrong_direction~%bool cartesian_spline_motion_generator_violation~%bool joint_via_motion_generator_planning_joint_limit_violation~%bool base_acceleration_initialization_timeout~%bool base_acceleration_invalid_reading~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint (revolute or prismatic) is defined by:~%#  * the position of the joint (rad or m),~%#  * the velocity of the joint (rad/s or m/s) and ~%#  * the effort that is applied in the joint (Nm or N).~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MultiCameraSynchronizedObservation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'camera_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'images) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gripper_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MultiCameraSynchronizedObservation>))
  "Converts a ROS message object to a list"
  (cl:list 'MultiCameraSynchronizedObservation
    (cl:cons ':header (header msg))
    (cl:cons ':camera_names (camera_names msg))
    (cl:cons ':images (images msg))
    (cl:cons ':state (state msg))
    (cl:cons ':gripper_state (gripper_state msg))
))
