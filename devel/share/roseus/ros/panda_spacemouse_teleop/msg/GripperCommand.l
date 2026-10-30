;; Auto-generated. Do not edit!


(when (boundp 'panda_spacemouse_teleop::GripperCommand)
  (if (not (find-package "PANDA_SPACEMOUSE_TELEOP"))
    (make-package "PANDA_SPACEMOUSE_TELEOP"))
  (shadow 'GripperCommand (find-package "PANDA_SPACEMOUSE_TELEOP")))
(unless (find-package "PANDA_SPACEMOUSE_TELEOP::GRIPPERCOMMAND")
  (make-package "PANDA_SPACEMOUSE_TELEOP::GRIPPERCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude GripperCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*OPEN*" (find-package "PANDA_SPACEMOUSE_TELEOP::GRIPPERCOMMAND"))
(shadow '*OPEN* (find-package "PANDA_SPACEMOUSE_TELEOP::GRIPPERCOMMAND"))
(defconstant panda_spacemouse_teleop::GripperCommand::*OPEN* 0)
(intern "*GRASP*" (find-package "PANDA_SPACEMOUSE_TELEOP::GRIPPERCOMMAND"))
(shadow '*GRASP* (find-package "PANDA_SPACEMOUSE_TELEOP::GRIPPERCOMMAND"))
(defconstant panda_spacemouse_teleop::GripperCommand::*GRASP* 1)

(defun panda_spacemouse_teleop::GripperCommand-to-symbol (const)
  (cond
        ((= const 0) 'panda_spacemouse_teleop::GripperCommand::*OPEN*)
        ((= const 1) 'panda_spacemouse_teleop::GripperCommand::*GRASP*)
        (t nil)))

(defclass panda_spacemouse_teleop::GripperCommand
  :super ros::object
  :slots (_header _command _width _speed _force _epsilon_inner _epsilon_outer ))

(defmethod panda_spacemouse_teleop::GripperCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:command __command) 0)
    ((:width __width) 0.0)
    ((:speed __speed) 0.0)
    ((:force __force) 0.0)
    ((:epsilon_inner __epsilon_inner) 0.0)
    ((:epsilon_outer __epsilon_outer) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _command (round __command))
   (setq _width (float __width))
   (setq _speed (float __speed))
   (setq _force (float __force))
   (setq _epsilon_inner (float __epsilon_inner))
   (setq _epsilon_outer (float __epsilon_outer))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:force
   (&optional __force)
   (if __force (setq _force __force)) _force)
  (:epsilon_inner
   (&optional __epsilon_inner)
   (if __epsilon_inner (setq _epsilon_inner __epsilon_inner)) _epsilon_inner)
  (:epsilon_outer
   (&optional __epsilon_outer)
   (if __epsilon_outer (setq _epsilon_outer __epsilon_outer)) _epsilon_outer)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _command
    1
    ;; float64 _width
    8
    ;; float64 _speed
    8
    ;; float64 _force
    8
    ;; float64 _epsilon_inner
    8
    ;; float64 _epsilon_outer
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _command
       (write-byte _command s)
     ;; float64 _width
       (sys::poke _width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _force
       (sys::poke _force (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _epsilon_inner
       (sys::poke _epsilon_inner (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _epsilon_outer
       (sys::poke _epsilon_outer (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _command
     (setq _command (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _width
     (setq _width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _speed
     (setq _speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _force
     (setq _force (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _epsilon_inner
     (setq _epsilon_inner (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _epsilon_outer
     (setq _epsilon_outer (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get panda_spacemouse_teleop::GripperCommand :md5sum-) "409a402d2a8c08a8d67a9748adb6125d")
(setf (get panda_spacemouse_teleop::GripperCommand :datatype-) "panda_spacemouse_teleop/GripperCommand")
(setf (get panda_spacemouse_teleop::GripperCommand :definition-)
      "std_msgs/Header header

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

")



(provide :panda_spacemouse_teleop/GripperCommand "409a402d2a8c08a8d67a9748adb6125d")


