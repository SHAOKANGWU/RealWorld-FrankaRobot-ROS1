; Auto-generated. Do not edit!


(cl:in-package panda_custom_controllers-msg)


;//! \htmlinclude JointPositionCommand.msg.html

(cl:defclass <JointPositionCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
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

(cl:defclass JointPositionCommand (<JointPositionCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointPositionCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointPositionCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_custom_controllers-msg:<JointPositionCommand> is deprecated: use panda_custom_controllers-msg:JointPositionCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <JointPositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_custom_controllers-msg:header-val is deprecated.  Use panda_custom_controllers-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'joint_positions-val :lambda-list '(m))
(cl:defmethod joint_positions-val ((m <JointPositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_custom_controllers-msg:joint_positions-val is deprecated.  Use panda_custom_controllers-msg:joint_positions instead.")
  (joint_positions m))

(cl:ensure-generic-function 'gripper_grasp-val :lambda-list '(m))
(cl:defmethod gripper_grasp-val ((m <JointPositionCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_custom_controllers-msg:gripper_grasp-val is deprecated.  Use panda_custom_controllers-msg:gripper_grasp instead.")
  (gripper_grasp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointPositionCommand>) ostream)
  "Serializes a message object of type '<JointPositionCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointPositionCommand>) istream)
  "Deserializes a message object of type '<JointPositionCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointPositionCommand>)))
  "Returns string type for a message object of type '<JointPositionCommand>"
  "panda_custom_controllers/JointPositionCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointPositionCommand)))
  "Returns string type for a message object of type 'JointPositionCommand"
  "panda_custom_controllers/JointPositionCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointPositionCommand>)))
  "Returns md5sum for a message object of type '<JointPositionCommand>"
  "976e7b5d3795a7b51c8c2bf1d2207512")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointPositionCommand)))
  "Returns md5sum for a message object of type 'JointPositionCommand"
  "976e7b5d3795a7b51c8c2bf1d2207512")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointPositionCommand>)))
  "Returns full string definition for message of type '<JointPositionCommand>"
  (cl:format cl:nil "std_msgs/Header header~%float64[7] joint_positions~%bool gripper_grasp~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointPositionCommand)))
  "Returns full string definition for message of type 'JointPositionCommand"
  (cl:format cl:nil "std_msgs/Header header~%float64[7] joint_positions~%bool gripper_grasp~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointPositionCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_positions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointPositionCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'JointPositionCommand
    (cl:cons ':header (header msg))
    (cl:cons ':joint_positions (joint_positions msg))
    (cl:cons ':gripper_grasp (gripper_grasp msg))
))
