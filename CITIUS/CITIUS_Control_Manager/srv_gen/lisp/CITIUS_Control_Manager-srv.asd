
(cl:in-package :asdf)

(defsystem "CITIUS_Control_Manager-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "srv_rearcam" :depends-on ("_package_srv_rearcam"))
    (:file "_package_srv_rearcam" :depends-on ("_package"))
    (:file "srv_frontcam" :depends-on ("_package_srv_frontcam"))
    (:file "_package_srv_frontcam" :depends-on ("_package"))
    (:file "srv_electric" :depends-on ("_package_srv_electric"))
    (:file "_package_srv_electric" :depends-on ("_package"))
    (:file "srv_vehicle" :depends-on ("_package_srv_vehicle"))
    (:file "_package_srv_vehicle" :depends-on ("_package"))
    (:file "srv_status" :depends-on ("_package_srv_status"))
    (:file "_package_srv_status" :depends-on ("_package"))
  ))