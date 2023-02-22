;; Auto-generated. Do not edit!


(when (boundp 'phidgets_msgs::EncoderDecimatedSpeed)
  (if (not (find-package "PHIDGETS_MSGS"))
    (make-package "PHIDGETS_MSGS"))
  (shadow 'EncoderDecimatedSpeed (find-package "PHIDGETS_MSGS")))
(unless (find-package "PHIDGETS_MSGS::ENCODERDECIMATEDSPEED")
  (make-package "PHIDGETS_MSGS::ENCODERDECIMATEDSPEED"))

(in-package "ROS")
;;//! \htmlinclude EncoderDecimatedSpeed.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass phidgets_msgs::EncoderDecimatedSpeed
  :super ros::object
  :slots (_header _avr_speed ))

(defmethod phidgets_msgs::EncoderDecimatedSpeed
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:avr_speed __avr_speed) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _avr_speed (float __avr_speed))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:avr_speed
   (&optional __avr_speed)
   (if __avr_speed (setq _avr_speed __avr_speed)) _avr_speed)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _avr_speed
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _avr_speed
       (sys::poke _avr_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _avr_speed
     (setq _avr_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get phidgets_msgs::EncoderDecimatedSpeed :md5sum-) "20fbdbe041b6e052c8c414d50464f125")
(setf (get phidgets_msgs::EncoderDecimatedSpeed :datatype-) "phidgets_msgs/EncoderDecimatedSpeed")
(setf (get phidgets_msgs::EncoderDecimatedSpeed :definition-)
      "# Encoder averaged speed for a channel in a Phidgets High-Speed Encoder board
Header header
# Averaged (sliding window) speed estimation [rad/s]
float64 avr_speed

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



(provide :phidgets_msgs/EncoderDecimatedSpeed "20fbdbe041b6e052c8c414d50464f125")


