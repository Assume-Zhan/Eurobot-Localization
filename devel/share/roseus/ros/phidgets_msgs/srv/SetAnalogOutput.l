;; Auto-generated. Do not edit!


(when (boundp 'phidgets_msgs::SetAnalogOutput)
  (if (not (find-package "PHIDGETS_MSGS"))
    (make-package "PHIDGETS_MSGS"))
  (shadow 'SetAnalogOutput (find-package "PHIDGETS_MSGS")))
(unless (find-package "PHIDGETS_MSGS::SETANALOGOUTPUT")
  (make-package "PHIDGETS_MSGS::SETANALOGOUTPUT"))
(unless (find-package "PHIDGETS_MSGS::SETANALOGOUTPUTREQUEST")
  (make-package "PHIDGETS_MSGS::SETANALOGOUTPUTREQUEST"))
(unless (find-package "PHIDGETS_MSGS::SETANALOGOUTPUTRESPONSE")
  (make-package "PHIDGETS_MSGS::SETANALOGOUTPUTRESPONSE"))

(in-package "ROS")





(defclass phidgets_msgs::SetAnalogOutputRequest
  :super ros::object
  :slots (_index _voltage ))

(defmethod phidgets_msgs::SetAnalogOutputRequest
  (:init
   (&key
    ((:index __index) 0)
    ((:voltage __voltage) 0.0)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _voltage (float __voltage))
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:serialization-length
   ()
   (+
    ;; uint16 _index
    2
    ;; float64 _voltage
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _index
       (write-word _index s)
     ;; float64 _voltage
       (sys::poke _voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _index
     (setq _index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _voltage
     (setq _voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass phidgets_msgs::SetAnalogOutputResponse
  :super ros::object
  :slots (_success ))

(defmethod phidgets_msgs::SetAnalogOutputResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass phidgets_msgs::SetAnalogOutput
  :super ros::object
  :slots ())

(setf (get phidgets_msgs::SetAnalogOutput :md5sum-) "c0d7b329e28c7be8f18cb4c1bd42580f")
(setf (get phidgets_msgs::SetAnalogOutput :datatype-) "phidgets_msgs/SetAnalogOutput")
(setf (get phidgets_msgs::SetAnalogOutput :request) phidgets_msgs::SetAnalogOutputRequest)
(setf (get phidgets_msgs::SetAnalogOutput :response) phidgets_msgs::SetAnalogOutputResponse)

(defmethod phidgets_msgs::SetAnalogOutputRequest
  (:response () (instance phidgets_msgs::SetAnalogOutputResponse :init)))

(setf (get phidgets_msgs::SetAnalogOutputRequest :md5sum-) "c0d7b329e28c7be8f18cb4c1bd42580f")
(setf (get phidgets_msgs::SetAnalogOutputRequest :datatype-) "phidgets_msgs/SetAnalogOutputRequest")
(setf (get phidgets_msgs::SetAnalogOutputRequest :definition-)
      "# Sets the state of a digital output.

uint16 index   # index of the output to be set
float64 voltage
---
bool success

")

(setf (get phidgets_msgs::SetAnalogOutputResponse :md5sum-) "c0d7b329e28c7be8f18cb4c1bd42580f")
(setf (get phidgets_msgs::SetAnalogOutputResponse :datatype-) "phidgets_msgs/SetAnalogOutputResponse")
(setf (get phidgets_msgs::SetAnalogOutputResponse :definition-)
      "# Sets the state of a digital output.

uint16 index   # index of the output to be set
float64 voltage
---
bool success

")



(provide :phidgets_msgs/SetAnalogOutput "c0d7b329e28c7be8f18cb4c1bd42580f")


