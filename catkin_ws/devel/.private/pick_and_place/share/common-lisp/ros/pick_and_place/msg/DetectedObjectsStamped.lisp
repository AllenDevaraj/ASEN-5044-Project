; Auto-generated. Do not edit!


(cl:in-package pick_and_place-msg)


;//! \htmlinclude DetectedObjectsStamped.msg.html

(cl:defclass <DetectedObjectsStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detected_objects
    :reader detected_objects
    :initarg :detected_objects
    :type (cl:vector pick_and_place-msg:DetectedObject)
   :initform (cl:make-array 0 :element-type 'pick_and_place-msg:DetectedObject :initial-element (cl:make-instance 'pick_and_place-msg:DetectedObject))))
)

(cl:defclass DetectedObjectsStamped (<DetectedObjectsStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DetectedObjectsStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DetectedObjectsStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pick_and_place-msg:<DetectedObjectsStamped> is deprecated: use pick_and_place-msg:DetectedObjectsStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DetectedObjectsStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick_and_place-msg:header-val is deprecated.  Use pick_and_place-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detected_objects-val :lambda-list '(m))
(cl:defmethod detected_objects-val ((m <DetectedObjectsStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick_and_place-msg:detected_objects-val is deprecated.  Use pick_and_place-msg:detected_objects instead.")
  (detected_objects m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DetectedObjectsStamped>) ostream)
  "Serializes a message object of type '<DetectedObjectsStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detected_objects))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detected_objects))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DetectedObjectsStamped>) istream)
  "Deserializes a message object of type '<DetectedObjectsStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detected_objects) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detected_objects)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pick_and_place-msg:DetectedObject))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DetectedObjectsStamped>)))
  "Returns string type for a message object of type '<DetectedObjectsStamped>"
  "pick_and_place/DetectedObjectsStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DetectedObjectsStamped)))
  "Returns string type for a message object of type 'DetectedObjectsStamped"
  "pick_and_place/DetectedObjectsStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DetectedObjectsStamped>)))
  "Returns md5sum for a message object of type '<DetectedObjectsStamped>"
  "94f8531a21eceed11f32889486790c82")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DetectedObjectsStamped)))
  "Returns md5sum for a message object of type 'DetectedObjectsStamped"
  "94f8531a21eceed11f32889486790c82")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DetectedObjectsStamped>)))
  "Returns full string definition for message of type '<DetectedObjectsStamped>"
  (cl:format cl:nil "std_msgs/Header header~%pick_and_place/DetectedObject[] detected_objects~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pick_and_place/DetectedObject~%float64 x_world~%float64 y_world~%float64 z_world~%float64 width~%float64 length~%float64 height~%string color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DetectedObjectsStamped)))
  "Returns full string definition for message of type 'DetectedObjectsStamped"
  (cl:format cl:nil "std_msgs/Header header~%pick_and_place/DetectedObject[] detected_objects~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: pick_and_place/DetectedObject~%float64 x_world~%float64 y_world~%float64 z_world~%float64 width~%float64 length~%float64 height~%string color~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DetectedObjectsStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detected_objects) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DetectedObjectsStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'DetectedObjectsStamped
    (cl:cons ':header (header msg))
    (cl:cons ':detected_objects (detected_objects msg))
))
