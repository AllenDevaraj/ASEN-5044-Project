
(cl:in-package :asdf)

(defsystem "pick_and_place-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DetectedObject" :depends-on ("_package_DetectedObject"))
    (:file "_package_DetectedObject" :depends-on ("_package"))
    (:file "DetectedObjectsStamped" :depends-on ("_package_DetectedObjectsStamped"))
    (:file "_package_DetectedObjectsStamped" :depends-on ("_package"))
  ))