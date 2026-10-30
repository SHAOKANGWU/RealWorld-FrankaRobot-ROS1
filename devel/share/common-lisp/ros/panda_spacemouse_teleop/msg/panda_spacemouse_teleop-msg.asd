
(cl:in-package :asdf)

(defsystem "panda_spacemouse_teleop-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GripperCommand" :depends-on ("_package_GripperCommand"))
    (:file "_package_GripperCommand" :depends-on ("_package"))
  ))