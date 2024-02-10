
(cl:in-package :asdf)

(defsystem "gps_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Customgps" :depends-on ("_package_Customgps"))
    (:file "_package_Customgps" :depends-on ("_package"))
  ))