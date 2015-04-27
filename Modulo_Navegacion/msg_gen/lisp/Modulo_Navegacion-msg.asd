
(cl:in-package :asdf)

(defsystem "Modulo_Navegacion-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
               :sensor_msgs-msg
               :tf-msg
)
  :components ((:file "_package")
    (:file "msg_mode" :depends-on ("_package_msg_mode"))
    (:file "_package_msg_mode" :depends-on ("_package"))
    (:file "msg_error" :depends-on ("_package_msg_error"))
    (:file "_package_msg_error" :depends-on ("_package"))
    (:file "msg_laser" :depends-on ("_package_msg_laser"))
    (:file "_package_msg_laser" :depends-on ("_package"))
    (:file "msg_gps" :depends-on ("_package_msg_gps"))
    (:file "_package_msg_gps" :depends-on ("_package"))
    (:file "msg_gest_navegacion" :depends-on ("_package_msg_gest_navegacion"))
    (:file "_package_msg_gest_navegacion" :depends-on ("_package"))
    (:file "msg_waypoints" :depends-on ("_package_msg_waypoints"))
    (:file "_package_msg_waypoints" :depends-on ("_package"))
    (:file "msg_module_enable" :depends-on ("_package_msg_module_enable"))
    (:file "_package_msg_module_enable" :depends-on ("_package"))
  ))