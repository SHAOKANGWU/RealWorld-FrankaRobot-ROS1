; Auto-generated. Do not edit!


(cl:in-package panda_custom_controllers-msg)


;//! \htmlinclude CartesianPoseCommand.msg.html

(cl:defclass <CartesianPoseCommand> (roslisp-msg-protocol:ros-message)
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
   (gripper_grasp
    :reader gripper_grasp
    :initarg :gripper_grasp
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CartesianPoseCommand (<CartesianPoseCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CartesianPoseCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CartesianPoseCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name panda_custom_controllers-msg:<CartesianPoseCommand> is deprecated: use panda_custom_controllers-msg:CartesianPoseCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CartesianPoseCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_custom_controllers-msg:header-val is deprecated.  Use panda_custom_controllers-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <CartesianPoseCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_custom_controllers-msg:pose-val is deprecated.  Use panda_custom_controllers-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'gripper_grasp-val :lambda-list '(m))
(cl:defmethod gripper_grasp-val ((m <CartesianPoseCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader panda_custom_controllers-msg:gripper_grasp-val is deprecated.  Use panda_custom_controllers-msg:gripper_grasp instead.")
  (gripper_grasp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CartesianPoseCommand>) ostream)
  "Serializes a message object of type '<CartesianPoseCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gripper_grasp) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CartesianPoseCommand>) istream)
  "Deserializes a message object of type '<CartesianPoseCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
    (cl:setf (cl:slot-value msg 'gripper_grasp) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CartesianPoseCommand>)))
  "Returns string type for a message object of type '<CartesianPoseCommand>"
  "panda_custom_controllers/CartesianPoseCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CartesianPoseCommand)))
  "Returns string type for a message object of type 'CartesianPoseCommand"
  "panda_custom_controllers/CartesianPoseCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CartesianPoseCommand>)))
  "Returns md5sum for a message object of type '<CartesianPoseCommand>"
  "1f89f6c1dd45bd7e2a9eb4013f35ce79")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CartesianPoseCommand)))
  "Returns md5sum for a message object of type 'CartesianPoseCommand"
  "1f89f6c1dd45bd7e2a9eb4013f35ce79")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CartesianPoseCommand>)))
  "Returns full string definition for message of type '<CartesianPoseCommand>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose pose~%bool gripper_grasp~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CartesianPoseCommand)))
  "Returns full string definition for message of type 'CartesianPoseCommand"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose pose~%bool gripper_grasp~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CartesianPoseCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CartesianPoseCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'CartesianPoseCommand
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':gripper_grasp (gripper_grasp msg))
))
