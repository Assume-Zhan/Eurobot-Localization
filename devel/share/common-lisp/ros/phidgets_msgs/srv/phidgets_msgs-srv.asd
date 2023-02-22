
(cl:in-package :asdf)

(defsystem "phidgets_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetAnalogOutput" :depends-on ("_package_SetAnalogOutput"))
    (:file "_package_SetAnalogOutput" :depends-on ("_package"))
    (:file "SetDigitalOutput" :depends-on ("_package_SetDigitalOutput"))
    (:file "_package_SetDigitalOutput" :depends-on ("_package"))
  ))