
(cl:in-package :asdf)

(defsystem "nuturtlebot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SensorData" :depends-on ("_package_SensorData"))
    (:file "_package_SensorData" :depends-on ("_package"))
    (:file "WheelCommands" :depends-on ("_package_WheelCommands"))
    (:file "_package_WheelCommands" :depends-on ("_package"))
  ))