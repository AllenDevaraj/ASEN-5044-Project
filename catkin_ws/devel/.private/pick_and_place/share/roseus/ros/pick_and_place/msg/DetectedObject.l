;; Auto-generated. Do not edit!


(when (boundp 'pick_and_place::DetectedObject)
  (if (not (find-package "PICK_AND_PLACE"))
    (make-package "PICK_AND_PLACE"))
  (shadow 'DetectedObject (find-package "PICK_AND_PLACE")))
(unless (find-package "PICK_AND_PLACE::DETECTEDOBJECT")
  (make-package "PICK_AND_PLACE::DETECTEDOBJECT"))

(in-package "ROS")
;;//! \htmlinclude DetectedObject.msg.html


(defclass pick_and_place::DetectedObject
  :super ros::object
  :slots (_x_world _y_world _z_world _width _length _height _color ))

(defmethod pick_and_place::DetectedObject
  (:init
   (&key
    ((:x_world __x_world) 0.0)
    ((:y_world __y_world) 0.0)
    ((:z_world __z_world) 0.0)
    ((:width __width) 0.0)
    ((:length __length) 0.0)
    ((:height __height) 0.0)
    ((:color __color) "")
    )
   (send-super :init)
   (setq _x_world (float __x_world))
   (setq _y_world (float __y_world))
   (setq _z_world (float __z_world))
   (setq _width (float __width))
   (setq _length (float __length))
   (setq _height (float __height))
   (setq _color (string __color))
   self)
  (:x_world
   (&optional __x_world)
   (if __x_world (setq _x_world __x_world)) _x_world)
  (:y_world
   (&optional __y_world)
   (if __y_world (setq _y_world __y_world)) _y_world)
  (:z_world
   (&optional __z_world)
   (if __z_world (setq _z_world __z_world)) _z_world)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:serialization-length
   ()
   (+
    ;; float64 _x_world
    8
    ;; float64 _y_world
    8
    ;; float64 _z_world
    8
    ;; float64 _width
    8
    ;; float64 _length
    8
    ;; float64 _height
    8
    ;; string _color
    4 (length _color)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x_world
       (sys::poke _x_world (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_world
       (sys::poke _y_world (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_world
       (sys::poke _z_world (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _width
       (sys::poke _width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _length
       (sys::poke _length (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _height
       (sys::poke _height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _color
       (write-long (length _color) s) (princ _color s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x_world
     (setq _x_world (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_world
     (setq _y_world (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_world
     (setq _z_world (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _width
     (setq _width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _length
     (setq _length (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _height
     (setq _height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _color
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _color (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get pick_and_place::DetectedObject :md5sum-) "167a85e1fd80883aabe7acad7edd6a85")
(setf (get pick_and_place::DetectedObject :datatype-) "pick_and_place/DetectedObject")
(setf (get pick_and_place::DetectedObject :definition-)
      "float64 x_world
float64 y_world
float64 z_world
float64 width
float64 length
float64 height
string color

")



(provide :pick_and_place/DetectedObject "167a85e1fd80883aabe7acad7edd6a85")


