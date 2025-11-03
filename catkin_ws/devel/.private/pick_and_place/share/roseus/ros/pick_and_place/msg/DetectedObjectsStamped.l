;; Auto-generated. Do not edit!


(when (boundp 'pick_and_place::DetectedObjectsStamped)
  (if (not (find-package "PICK_AND_PLACE"))
    (make-package "PICK_AND_PLACE"))
  (shadow 'DetectedObjectsStamped (find-package "PICK_AND_PLACE")))
(unless (find-package "PICK_AND_PLACE::DETECTEDOBJECTSSTAMPED")
  (make-package "PICK_AND_PLACE::DETECTEDOBJECTSSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude DetectedObjectsStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pick_and_place::DetectedObjectsStamped
  :super ros::object
  :slots (_header _detected_objects ))

(defmethod pick_and_place::DetectedObjectsStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detected_objects __detected_objects) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _detected_objects __detected_objects)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detected_objects
   (&rest __detected_objects)
   (if (keywordp (car __detected_objects))
       (send* _detected_objects __detected_objects)
     (progn
       (if __detected_objects (setq _detected_objects (car __detected_objects)))
       _detected_objects)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pick_and_place/DetectedObject[] _detected_objects
    (apply #'+ (send-all _detected_objects :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pick_and_place/DetectedObject[] _detected_objects
     (write-long (length _detected_objects) s)
     (dolist (elem _detected_objects)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pick_and_place/DetectedObject[] _detected_objects
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detected_objects (let (r) (dotimes (i n) (push (instance pick_and_place::DetectedObject :init) r)) r))
     (dolist (elem- _detected_objects)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get pick_and_place::DetectedObjectsStamped :md5sum-) "94f8531a21eceed11f32889486790c82")
(setf (get pick_and_place::DetectedObjectsStamped :datatype-) "pick_and_place/DetectedObjectsStamped")
(setf (get pick_and_place::DetectedObjectsStamped :definition-)
      "std_msgs/Header header
pick_and_place/DetectedObject[] detected_objects

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

================================================================================
MSG: pick_and_place/DetectedObject
float64 x_world
float64 y_world
float64 z_world
float64 width
float64 length
float64 height
string color

")



(provide :pick_and_place/DetectedObjectsStamped "94f8531a21eceed11f32889486790c82")


