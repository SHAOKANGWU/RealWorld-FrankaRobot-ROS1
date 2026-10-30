; Auto-generated. Do not edit!


(cl:in-package panda_custom_controllers-msg)


;//! \htmlinclude NullSpaceCommand.msg.html

(cl:defclass <NullSpaceCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (joint_positions
    :reader joint_positions
    :initarg :joint_positions
    :type (cl:vector cl:float)
   :initform (cl:make-array 7 :element-type 'cl:float :initial-element 0.0))
   (gripper_grasp
    :reader gripper_grasp
    :initarg :gripper_grasp
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass NullSpaceCommand (<NullSpaceCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NullSpaceCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NullSpaceCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_custom_controllers-msg:<NullSpaceCommand> is deprecated: use panda_custom_controllers-msg:NullSpaceCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <NullSpaceCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_custom_controllers-msg:header-val is deprecated.  Use panda_custom_controllers-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <NullSpaceCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_custom_controllers-msg:pose-val is deprecated.  Use panda_custom_controllers-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'joint_positions-val :lambda-list '(m))
(cl:defmethod joint_positions-val ((m <NullSpaceCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_custom_controllers-msg:joint_positions-val is deprecated.  Use panda_custom_controllers-msg:joint_positions instead.")
  (joint_positions m))

(cl:ensure-generic-function 'gripper_grasp-val :lambda-list '(m))
(cl:defmethod gripper_grasp-val ((m <NullSpaceCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_custom_controllers-msg:gripper_grasp-val is deprecated.  Use panda_custom_controllers-msg:gripper_grasp instead.")
  (gripper_grasp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NullSpaceCommand>) ostream)
  "Serializes a message object of type '<NullSpaceCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'joint_positions))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gripper_grasp) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NullSpaceCommand>) istream)
  "Deserializes a message object of type '<NullSpaceCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (cl:setf (cl:slot-value msg 'joint_positions) (cl:make-array 7))
  (cl:let ((vals (cl:slot-value msg 'joint_positions)))
    (cl:dotimes (i 7)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:setf (cl:slot-value msg 'gripper_grasp) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NullSpaceCommand>)))
  "Returns string type for a message object of type '<NullSpaceCommand>"
  "panda_custom_controllers/NullSpaceCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NullSpaceCommand)))
  "Returns string type for a message object of type 'NullSpaceCommand"
  "panda_custom_controllers/NullSpaceCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NullSpaceCommand>)))
  "Returns md5sum for a message object of type '<NullSpaceCommand>"
  "ac7f965b4f3f525107f6071ff3534376")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NullSpaceCommand)))
  "Returns md5sum for a message object of type 'NullSpaceCommand"
  "ac7f965b4f3f525107f6071ff3534376")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NullSpaceCommand>)))
  "Returns full string definition for message of type '<NullSpaceCommand>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose pose~%float64[7] joint_positions~%bool gripper_grasp~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NullSpaceCommand)))
  "Returns full string definition for message of type 'NullSpaceCommand"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose pose~%float64[7] joint_positions~%bool gripper_grasp~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NullSpaceCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NullSpaceCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'NullSpaceCommand
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':joint_positions (joint_positions msg))
    (cl:cons ':gripper_grasp (gripper_grasp msg))
))
