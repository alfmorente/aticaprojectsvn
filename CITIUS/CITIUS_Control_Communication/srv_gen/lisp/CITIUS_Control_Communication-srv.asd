
(cl:in-package :asdf)

(defsystem "CITIUS_Control_Communication-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "srv_zoomCommand" :depends-on ("_package_srv_zoomCommand"))
    (:file "_package_srv_zoomCommand" :depends-on ("_package"))
    (:file "srv_zoomDirect" :depends-on ("_package_srv_zoomDirect"))
    (:file "_package_srv_zoomDirect" :depends-on ("_package"))
    (:file "srv_dzoom" :depends-on ("_package_srv_dzoom"))
    (:file "_package_srv_dzoom" :depends-on ("_package"))
    (:file "srv_panAbsolutePosition" :depends-on ("_package_srv_panAbsolutePosition"))
    (:file "_package_srv_panAbsolutePosition" :depends-on ("_package"))
    (:file "srv_shoot" :depends-on ("_package_srv_shoot"))
    (:file "_package_srv_shoot" :depends-on ("_package"))
    (:file "srv_focusDirect" :depends-on ("_package_srv_focusDirect"))
    (:file "_package_srv_focusDirect" :depends-on ("_package"))
    (:file "srv_tiltRate" :depends-on ("_package_srv_tiltRate"))
    (:file "_package_srv_tiltRate" :depends-on ("_package"))
    (:file "srv_autofocusMode" :depends-on ("_package_srv_autofocusMode"))
    (:file "_package_srv_autofocusMode" :depends-on ("_package"))
    (:file "srv_polarity" :depends-on ("_package_srv_polarity"))
    (:file "_package_srv_polarity" :depends-on ("_package"))
    (:file "srv_panRate" :depends-on ("_package_srv_panRate"))
    (:file "_package_srv_panRate" :depends-on ("_package"))
    (:file "srv_vehicleStatus" :depends-on ("_package_srv_vehicleStatus"))
    (:file "_package_srv_vehicleStatus" :depends-on ("_package"))
    (:file "srv_tiltAbsolutePosition" :depends-on ("_package_srv_tiltAbsolutePosition"))
    (:file "_package_srv_tiltAbsolutePosition" :depends-on ("_package"))
  ))