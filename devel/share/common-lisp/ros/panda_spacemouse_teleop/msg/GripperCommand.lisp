; Auto-generated. Do not edit!


(cl:in-package panda_spacemouse_teleop-msg)


;//! \htmlinclude GripperCommand.msg.html

(cl:defclass <GripperCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (force
    :reader force
    :initarg :force
    :type cl:float
    :initform 0.0)
   (epsilon_inner
    :reader epsilon_inner
    :initarg :epsilon_inner
    :type cl:float
    :initform 0.0)
   (epsilon_outer
    :reader epsilon_outer
    :initarg :epsilon_outer
    :type cl:float
    :initform 0.0))
)

(cl:defclass GripperCommand (<GripperCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GripperCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GripperCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_spacemouse_teleop-msg:<GripperCommand> is deprecated: use panda_spacemouse_teleop-msg:GripperCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GripperCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_spacemouse_teleop-msg:header-val is deprecated.  Use panda_spacemouse_teleop-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <GripperCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_spacemouse_teleop-msg:command-val is deprecated.  Use panda_spacemouse_teleop-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <GripperCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_spacemouse_teleop-msg:width-val is deprecated.  Use panda_spacemouse_teleop-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <GripperCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_spacemouse_teleop-msg:speed-val is deprecated.  Use panda_spacemouse_teleop-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <GripperCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_spacemouse_teleop-msg:force-val is deprecated.  Use panda_spacemouse_teleop-msg:force instead.")
  (force m))

(cl:ensure-generic-function 'epsilon_inner-val :lambda-list '(m))
(cl:defmethod epsilon_inner-val ((m <GripperCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_spacemouse_teleop-msg:epsilon_inner-val is deprecated.  Use panda_spacemouse_teleop-msg:epsilon_inner instead.")
  (epsilon_inner m))

(cl:ensure-generic-function 'epsilon_outer-val :lambda-list '(m))
(cl:defmethod epsilon_outer-val ((m <GripperCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_spacemouse_teleop-msg:epsilon_outer-val is deprecated.  Use panda_spacemouse_teleop-msg:epsilon_outer instead.")
  (epsilon_outer m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GripperCommand>)))
    "Constants for message type '<GripperCommand>"
  '((:OPEN . 0)
    (:GRASP . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GripperCommand)))
    "Constants for message type 'GripperCommand"
  '((:OPEN . 0)
    (:GRASP . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GripperCommand>) ostream)
  "Serializes a message object of type '<GripperCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'epsilon_inner))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'epsilon_outer))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GripperCommand>) istream)
  "Deserializes a message object of type '<GripperCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'epsilon_inner) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'epsilon_outer) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GripperCommand>)))
  "Returns string type for a message object of type '<GripperCommand>"
  "panda_spacemouse_teleop/GripperCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GripperCommand)))
  "Returns string type for a message object of type 'GripperCommand"
  "panda_spacemouse_teleop/GripperCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GripperCommand>)))
  "Returns md5sum for a message object of type '<GripperCommand>"
  "409a402d2a8c08a8d67a9748adb6125d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GripperCommand)))
  "Returns md5sum for a message object of type 'GripperCommand"
  "409a402d2a8c08a8d67a9748adb6125d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GripperCommand>)))
  "Returns full string definition for message of type '<GripperCommand>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 OPEN=0~%uint8 GRASP=1~%uint8 command~%~%float64 width~%float64 speed~%float64 force~%float64 epsilon_inner~%float64 epsilon_outer~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GripperCommand)))
  "Returns full string definition for message of type 'GripperCommand"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 OPEN=0~%uint8 GRASP=1~%uint8 command~%~%float64 width~%float64 speed~%float64 force~%float64 epsilon_inner~%float64 epsilon_outer~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GripperCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GripperCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'GripperCommand
    (cl:cons ':header (header msg))
    (cl:cons ':command (command msg))
    (cl:cons ':width (width msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':force (force msg))
    (cl:cons ':epsilon_inner (epsilon_inner msg))
    (cl:cons ':epsilon_outer (epsilon_outer msg))
))
