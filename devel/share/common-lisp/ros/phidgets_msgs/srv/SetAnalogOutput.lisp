; Auto-generated. Do not edit!


(cl:in-package phidgets_msgs-srv)


;//! \htmlinclude SetAnalogOutput-request.msg.html

(cl:defclass <SetAnalogOutput-request> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0)
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetAnalogOutput-request (<SetAnalogOutput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAnalogOutput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAnalogOutput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phidgets_msgs-srv:<SetAnalogOutput-request> is deprecated: use phidgets_msgs-srv:SetAnalogOutput-request instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <SetAnalogOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phidgets_msgs-srv:index-val is deprecated.  Use phidgets_msgs-srv:index instead.")
  (index m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <SetAnalogOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phidgets_msgs-srv:voltage-val is deprecated.  Use phidgets_msgs-srv:voltage instead.")
  (voltage m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAnalogOutput-request>) ostream)
  "Serializes a message object of type '<SetAnalogOutput-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAnalogOutput-request>) istream)
  "Deserializes a message object of type '<SetAnalogOutput-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'voltage) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAnalogOutput-request>)))
  "Returns string type for a service object of type '<SetAnalogOutput-request>"
  "phidgets_msgs/SetAnalogOutputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAnalogOutput-request)))
  "Returns string type for a service object of type 'SetAnalogOutput-request"
  "phidgets_msgs/SetAnalogOutputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAnalogOutput-request>)))
  "Returns md5sum for a message object of type '<SetAnalogOutput-request>"
  "c0d7b329e28c7be8f18cb4c1bd42580f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAnalogOutput-request)))
  "Returns md5sum for a message object of type 'SetAnalogOutput-request"
  "c0d7b329e28c7be8f18cb4c1bd42580f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAnalogOutput-request>)))
  "Returns full string definition for message of type '<SetAnalogOutput-request>"
  (cl:format cl:nil "# Sets the state of a digital output.~%~%uint16 index   # index of the output to be set~%float64 voltage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAnalogOutput-request)))
  "Returns full string definition for message of type 'SetAnalogOutput-request"
  (cl:format cl:nil "# Sets the state of a digital output.~%~%uint16 index   # index of the output to be set~%float64 voltage~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAnalogOutput-request>))
  (cl:+ 0
     2
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAnalogOutput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAnalogOutput-request
    (cl:cons ':index (index msg))
    (cl:cons ':voltage (voltage msg))
))
;//! \htmlinclude SetAnalogOutput-response.msg.html

(cl:defclass <SetAnalogOutput-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetAnalogOutput-response (<SetAnalogOutput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAnalogOutput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAnalogOutput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phidgets_msgs-srv:<SetAnalogOutput-response> is deprecated: use phidgets_msgs-srv:SetAnalogOutput-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetAnalogOutput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phidgets_msgs-srv:success-val is deprecated.  Use phidgets_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAnalogOutput-response>) ostream)
  "Serializes a message object of type '<SetAnalogOutput-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAnalogOutput-response>) istream)
  "Deserializes a message object of type '<SetAnalogOutput-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAnalogOutput-response>)))
  "Returns string type for a service object of type '<SetAnalogOutput-response>"
  "phidgets_msgs/SetAnalogOutputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAnalogOutput-response)))
  "Returns string type for a service object of type 'SetAnalogOutput-response"
  "phidgets_msgs/SetAnalogOutputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAnalogOutput-response>)))
  "Returns md5sum for a message object of type '<SetAnalogOutput-response>"
  "c0d7b329e28c7be8f18cb4c1bd42580f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAnalogOutput-response)))
  "Returns md5sum for a message object of type 'SetAnalogOutput-response"
  "c0d7b329e28c7be8f18cb4c1bd42580f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAnalogOutput-response>)))
  "Returns full string definition for message of type '<SetAnalogOutput-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAnalogOutput-response)))
  "Returns full string definition for message of type 'SetAnalogOutput-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAnalogOutput-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAnalogOutput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAnalogOutput-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAnalogOutput)))
  'SetAnalogOutput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAnalogOutput)))
  'SetAnalogOutput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAnalogOutput)))
  "Returns string type for a service object of type '<SetAnalogOutput>"
  "phidgets_msgs/SetAnalogOutput")