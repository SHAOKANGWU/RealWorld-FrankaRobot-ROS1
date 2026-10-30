
(cl:in-package :asdf)

(defsystem "observation_receiver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :franka_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "MultiCameraSynchronizedObservation" :depends-on ("_package_MultiCameraSynchronizedObservation"))
    (:file "_package_MultiCameraSynchronizedObservation" :depends-on ("_package"))
    (:file "SynchronizedObservation" :depends-on ("_package_SynchronizedObservation"))
    (:file "_package_SynchronizedObservation" :depends-on ("_package"))
  ))