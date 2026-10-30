;; Auto-generated. Do not edit!


(when (boundp 'panda_custom_controllers::NullSpaceCommand)
  (if (not (find-package "PANDA_CUSTOM_CONTROLLERS"))
    (make-package "PANDA_CUSTOM_CONTROLLERS"))
  (shadow 'NullSpaceCommand (find-package "PANDA_CUSTOM_CONTROLLERS")))
(unless (find-package "PANDA_CUSTOM_CONTROLLERS::NULLSPACECOMMAND")
  (make-package "PANDA_CUSTOM_CONTROLLERS::NULLSPACECOMMAND"))

(in-package "ROS")
;;//! \htmlinclude NullSpaceCommand.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass panda_custom_controllers::NullSpaceCommand
  :super ros::object
  :slots (_header _pose _joint_positions _gripper_grasp ))

(defmethod panda_custom_controllers::NullSpaceCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:joint_positions __joint_positions) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:gripper_grasp __gripper_grasp) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose __pose)
   (setq _joint_positions __joint_positions)
   (setq _gripper_grasp __gripper_grasp)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:joint_positions
   (&optional __joint_positions)
   (if __joint_positions (setq _joint_positions __joint_positions)) _joint_positions)
  (:gripper_grasp
   (&optional (__gripper_grasp :null))
   (if (not (eq __gripper_grasp :null)) (setq _gripper_grasp __gripper_grasp)) _gripper_grasp)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; float64[7] _joint_positions
    (* 8    7)
    ;; bool _gripper_grasp
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; float64[7] _joint_positions
     (dotimes (i 7)
       (sys::poke (elt _joint_positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _gripper_grasp
       (if _gripper_grasp (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; float64[7] _joint_positions
   (dotimes (i (length _joint_positions))
     (setf (elt _joint_positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _gripper_grasp
     (setq _gripper_grasp (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get panda_custom_controllers::NullSpaceCommand :md5sum-) "ac7f965b4f3f525107f6071ff3534376")
(setf (get panda_custom_controllers::NullSpaceCommand :datatype-) "panda_custom_controllers/NullSpaceCommand")
(setf (get panda_custom_controllers::NullSpaceCommand :definition-)
      "std_msgs/Header header
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

")



(provide :panda_custom_controllers/NullSpaceCommand "ac7f965b4f3f525107f6071ff3534376")


