; Auto-generated. Do not edit!


(cl:in-package phidgets_msgs-msg)


;//! \htmlinclude EncoderDecimatedSpeed.msg.html

(cl:defclass <EncoderDecimatedSpeed> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (avr_speed
    :reader avr_speed
    :initarg :avr_speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass EncoderDecimatedSpeed (<EncoderDecimatedSpeed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EncoderDecimatedSpeed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EncoderDecimatedSpeed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phidgets_msgs-msg:<EncoderDecimatedSpeed> is deprecated: use phidgets_msgs-msg:EncoderDecimatedSpeed instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EncoderDecimatedSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phidgets_msgs-msg:header-val is deprecated.  Use phidgets_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'avr_speed-val :lambda-list '(m))
(cl:defmethod avr_speed-val ((m <EncoderDecimatedSpeed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phidgets_msgs-msg:avr_speed-val is deprecated.  Use phidgets_msgs-msg:avr_speed instead.")
  (avr_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EncoderDecimatedSpeed>) ostream)
  "Serializes a message object of type '<EncoderDecimatedSpeed>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'avr_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EncoderDecimatedSpeed>) istream)
  "Deserializes a message object of type '<EncoderDecimatedSpeed>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'avr_speed) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EncoderDecimatedSpeed>)))
  "Returns string type for a message object of type '<EncoderDecimatedSpeed>"
  "phidgets_msgs/EncoderDecimatedSpeed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EncoderDecimatedSpeed)))
  "Returns string type for a message object of type 'EncoderDecimatedSpeed"
  "phidgets_msgs/EncoderDecimatedSpeed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EncoderDecimatedSpeed>)))
  "Returns md5sum for a message object of type '<EncoderDecimatedSpeed>"
  "20fbdbe041b6e052c8c414d50464f125")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EncoderDecimatedSpeed)))
  "Returns md5sum for a message object of type 'EncoderDecimatedSpeed"
  "20fbdbe041b6e052c8c414d50464f125")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EncoderDecimatedSpeed>)))
  "Returns full string definition for message of type '<EncoderDecimatedSpeed>"
  (cl:format cl:nil "# Encoder averaged speed for a channel in a Phidgets High-Speed Encoder board~%Header header~%# Averaged (sliding window) speed estimation [rad/s]~%float64 avr_speed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EncoderDecimatedSpeed)))
  "Returns full string definition for message of type 'EncoderDecimatedSpeed"
  (cl:format cl:nil "# Encoder averaged speed for a channel in a Phidgets High-Speed Encoder board~%Header header~%# Averaged (sliding window) speed estimation [rad/s]~%float64 avr_speed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EncoderDecimatedSpeed>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EncoderDecimatedSpeed>))
  "Converts a ROS message object to a list"
  (cl:list 'EncoderDecimatedSpeed
    (cl:cons ':header (header msg))
    (cl:cons ':avr_speed (avr_speed msg))
))
