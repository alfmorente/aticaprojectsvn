
(cl:in-package :asdf)

(defsystem "Modulo_Comunicaciones-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "msg_modo" :depends-on ("_package_msg_modo"))
    (:file "_package_msg_modo" :depends-on ("_package"))
    (:file "msg_camaras" :depends-on ("_package_msg_camaras"))
    (:file "_package_msg_camaras" :depends-on ("_package"))
    (:file "msg_waypoint" :depends-on ("_package_msg_waypoint"))
    (:file "_package_msg_waypoint" :depends-on ("_package"))
    (:file "msg_gps" :depends-on ("_package_msg_gps"))
    (:file "_package_msg_gps" :depends-on ("_package"))
    (:file "msg_com_teleoperado" :depends-on ("_package_msg_com_teleoperado"))
    (:file "_package_msg_com_teleoperado" :depends-on ("_package"))
    (:file "msg_errores" :depends-on ("_package_msg_errores"))
    (:file "_package_msg_errores" :depends-on ("_package"))
  ))