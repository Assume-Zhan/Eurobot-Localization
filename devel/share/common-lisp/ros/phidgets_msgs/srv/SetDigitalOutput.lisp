; Auto-generated. Do not edit!


(cl:in-package phidgets_msgs-srv)


;//! \htmlinclude SetDigitalOutput-request.msg.html

(cl:defclass <SetDigitalOutput-request> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetDigitalOutput-request (<SetDigitalOutput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDigitalOutput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDigitalOutput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phidgets_msgs-srv:<SetDigitalOutput-request> is deprecated: use phidgets_msgs-srv:SetDigitalOutput-request instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <SetDigitalOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phidgets_msgs-srv:index-val is deprecated.  Use phidgets_msgs-srv:index instead.")
  (index m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SetDigitalOutput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phidgets_msgs-srv:state-val is deprecated.  Use phidgets_msgs-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDigitalOutput-request>) ostream)
  "Serializes a message object of type '<SetDigitalOutput-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDigitalOutput-request>) istream)
  "Deserializes a message object of type '<SetDigitalOutput-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'index)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDigitalOutput-request>)))
  "Returns string type for a service object of type '<SetDigitalOutput-request>"
  "phidgets_msgs/SetDigitalOutputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDigitalOutput-request)))
  "Returns string type for a service object of type 'SetDigitalOutput-request"
  "phidgets_msgs/SetDigitalOutputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDigitalOutput-request>)))
  "Returns md5sum for a message object of type '<SetDigitalOutput-request>"
  "8496af00b3dd95e3884fd81d8e38f019")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDigitalOutput-request)))
  "Returns md5sum for a message object of type 'SetDigitalOutput-request"
  "8496af00b3dd95e3884fd81d8e38f019")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDigitalOutput-request>)))
  "Returns full string definition for message of type '<SetDigitalOutput-request>"
  (cl:format cl:nil "# Sets the state of a digital output.~%~%uint16 index   # index of the output to be set~%bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDigitalOutput-request)))
  "Returns full string definition for message of type 'SetDigitalOutput-request"
  (cl:format cl:nil "# Sets the state of a digital output.~%~%uint16 index   # index of the output to be set~%bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDigitalOutput-request>))
  (cl:+ 0
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDigitalOutput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDigitalOutput-request
    (cl:cons ':index (index msg))
    (cl:cons ':state (state msg))
))
;//! \htmlinclude SetDigitalOutput-response.msg.html

(cl:defclass <SetDigitalOutput-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetDigitalOutput-response (<SetDigitalOutput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDigitalOutput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDigitalOutput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phidgets_msgs-srv:<SetDigitalOutput-response> is deprecated: use phidgets_msgs-srv:SetDigitalOutput-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetDigitalOutput-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phidgets_msgs-srv:success-val is deprecated.  Use phidgets_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDigitalOutput-response>) ostream)
  "Serializes a message object of type '<SetDigitalOutput-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDigitalOutput-response>) istream)
  "Deserializes a message object of type '<SetDigitalOutput-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDigitalOutput-response>)))
  "Returns string type for a service object of type '<SetDigitalOutput-response>"
  "phidgets_msgs/SetDigitalOutputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDigitalOutput-response)))
  "Returns string type for a service object of type 'SetDigitalOutput-response"
  "phidgets_msgs/SetDigitalOutputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDigitalOutput-response>)))
  "Returns md5sum for a message object of type '<SetDigitalOutput-response>"
  "8496af00b3dd95e3884fd81d8e38f019")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDigitalOutput-response)))
  "Returns md5sum for a message object of type 'SetDigitalOutput-response"
  "8496af00b3dd95e3884fd81d8e38f019")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDigitalOutput-response>)))
  "Returns full string definition for message of type '<SetDigitalOutput-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDigitalOutput-response)))
  "Returns full string definition for message of type 'SetDigitalOutput-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDigitalOutput-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDigitalOutput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDigitalOutput-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetDigitalOutput)))
  'SetDigitalOutput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetDigitalOutput)))
  'SetDigitalOutput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDigitalOutput)))
  "Returns string type for a service object of type '<SetDigitalOutput>"
  "phidgets_msgs/SetDigitalOutput")