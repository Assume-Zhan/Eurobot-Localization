;; Auto-generated. Do not edit!


(when (boundp 'phidgets_msgs::SetDigitalOutput)
  (if (not (find-package "PHIDGETS_MSGS"))
    (make-package "PHIDGETS_MSGS"))
  (shadow 'SetDigitalOutput (find-package "PHIDGETS_MSGS")))
(unless (find-package "PHIDGETS_MSGS::SETDIGITALOUTPUT")
  (make-package "PHIDGETS_MSGS::SETDIGITALOUTPUT"))
(unless (find-package "PHIDGETS_MSGS::SETDIGITALOUTPUTREQUEST")
  (make-package "PHIDGETS_MSGS::SETDIGITALOUTPUTREQUEST"))
(unless (find-package "PHIDGETS_MSGS::SETDIGITALOUTPUTRESPONSE")
  (make-package "PHIDGETS_MSGS::SETDIGITALOUTPUTRESPONSE"))

(in-package "ROS")





(defclass phidgets_msgs::SetDigitalOutputRequest
  :super ros::object
  :slots (_index _state ))

(defmethod phidgets_msgs::SetDigitalOutputRequest
  (:init
   (&key
    ((:index __index) 0)
    ((:state __state) nil)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _state __state)
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:state
   (&optional (__state :null))
   (if (not (eq __state :null)) (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint16 _index
    2
    ;; bool _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _index
       (write-word _index s)
     ;; bool _state
       (if _state (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _index
     (setq _index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _state
     (setq _state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass phidgets_msgs::SetDigitalOutputResponse
  :super ros::object
  :slots (_success ))

(defmethod phidgets_msgs::SetDigitalOutputResponse
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

(defclass phidgets_msgs::SetDigitalOutput
  :super ros::object
  :slots ())

(setf (get phidgets_msgs::SetDigitalOutput :md5sum-) "8496af00b3dd95e3884fd81d8e38f019")
(setf (get phidgets_msgs::SetDigitalOutput :datatype-) "phidgets_msgs/SetDigitalOutput")
(setf (get phidgets_msgs::SetDigitalOutput :request) phidgets_msgs::SetDigitalOutputRequest)
(setf (get phidgets_msgs::SetDigitalOutput :response) phidgets_msgs::SetDigitalOutputResponse)

(defmethod phidgets_msgs::SetDigitalOutputRequest
  (:response () (instance phidgets_msgs::SetDigitalOutputResponse :init)))

(setf (get phidgets_msgs::SetDigitalOutputRequest :md5sum-) "8496af00b3dd95e3884fd81d8e38f019")
(setf (get phidgets_msgs::SetDigitalOutputRequest :datatype-) "phidgets_msgs/SetDigitalOutputRequest")
(setf (get phidgets_msgs::SetDigitalOutputRequest :definition-)
      "# Sets the state of a digital output.

uint16 index   # index of the output to be set
bool state
---
bool success

")

(setf (get phidgets_msgs::SetDigitalOutputResponse :md5sum-) "8496af00b3dd95e3884fd81d8e38f019")
(setf (get phidgets_msgs::SetDigitalOutputResponse :datatype-) "phidgets_msgs/SetDigitalOutputResponse")
(setf (get phidgets_msgs::SetDigitalOutputResponse :definition-)
      "# Sets the state of a digital output.

uint16 index   # index of the output to be set
bool state
---
bool success

")



(provide :phidgets_msgs/SetDigitalOutput "8496af00b3dd95e3884fd81d8e38f019")


