;; Auto-generated. Do not edit!


(when (boundp 'observation_receiver::SynchronizedObservation)
  (if (not (find-package "OBSERVATION_RECEIVER"))
    (make-package "OBSERVATION_RECEIVER"))
  (shadow 'SynchronizedObservation (find-package "OBSERVATION_RECEIVER")))
(unless (find-package "OBSERVATION_RECEIVER::SYNCHRONIZEDOBSERVATION")
  (make-package "OBSERVATION_RECEIVER::SYNCHRONIZEDOBSERVATION"))

(in-package "ROS")
;;//! \htmlinclude SynchronizedObservation.msg.html
(if (not (find-package "FRANKA_MSGS"))
  (ros::roseus-add-msgs "franka_msgs"))
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass observation_receiver::SynchronizedObservation
  :super ros::object
  :slots (_header _image _state _gripper_state ))

(defmethod observation_receiver::SynchronizedObservation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:image __image) (instance sensor_msgs::Image :init))
    ((:state __state) (instance franka_msgs::FrankaState :init))
    ((:gripper_state __gripper_state) (instance sensor_msgs::JointState :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _image __image)
   (setq _state __state)
   (setq _gripper_state __gripper_state)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:state
   (&rest __state)
   (if (keywordp (car __state))
       (send* _state __state)
     (progn
       (if __state (setq _state (car __state)))
       _state)))
  (:gripper_state
   (&rest __gripper_state)
   (if (keywordp (car __gripper_state))
       (send* _gripper_state __gripper_state)
     (progn
       (if __gripper_state (setq _gripper_state (car __gripper_state)))
       _gripper_state)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ;; franka_msgs/FrankaState _state
    (send _state :serialization-length)
    ;; sensor_msgs/JointState _gripper_state
    (send _gripper_state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;; franka_msgs/FrankaState _state
       (send _state :serialize s)
     ;; sensor_msgs/JointState _gripper_state
       (send _gripper_state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;; franka_msgs/FrankaState _state
     (send _state :deserialize buf ptr-) (incf ptr- (send _state :serialization-length))
   ;; sensor_msgs/JointState _gripper_state
     (send _gripper_state :deserialize buf ptr-) (incf ptr- (send _gripper_state :serialization-length))
   ;;
   self)
  )

(setf (get observation_receiver::SynchronizedObservation :md5sum-) "946f8d69db9a02477671cd93c5613135")
(setf (get observation_receiver::SynchronizedObservation :datatype-) "observation_receiver/SynchronizedObservation")
(setf (get observation_receiver::SynchronizedObservation :definition-)
      "std_msgs/Header header
sensor_msgs/Image image
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

")



(provide :observation_receiver/SynchronizedObservation "946f8d69db9a02477671cd93c5613135")


