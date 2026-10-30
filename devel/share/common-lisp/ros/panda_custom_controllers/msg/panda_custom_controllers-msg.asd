
(cl:in-package :asdf)

(defsystem "panda_custom_controllers-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CartesianPoseCommand" :depends-on ("_package_CartesianPoseCommand"))
    (:file "_package_CartesianPoseCommand" :depends-on ("_package"))
    (:file "JointPositionCommand" :depends-on ("_package_JointPositionCommand"))
    (:file "_package_JointPositionCommand" :depends-on ("_package"))
    (:file "NullSpaceCommand" :depends-on ("_package_NullSpaceCommand"))
    (:file "_package_NullSpaceCommand" :depends-on ("_package"))
  ))