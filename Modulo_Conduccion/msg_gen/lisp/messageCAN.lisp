; Auto-generated. Do not edit!


(cl:in-package Modulo_Conduccion-msg)


;//! \htmlinclude messageCAN.msg.html

(cl:defclass <messageCAN> (roslisp-msg-protocol:ros-message)
  ((senal
    :reader senal
    :initarg :senal
    :type cl:fixnum
    :initform 0)
   (opcion_etc1
    :reader opcion_etc1
    :initarg :opcion_etc1
    :type cl:string
    :initform "")
   (revoluciones_etc1
    :reader revoluciones_etc1
    :initarg :revoluciones_etc1
    :type cl:float
    :initform 0.0)
   (porcentaje_etc1
    :reader porcentaje_etc1
    :initarg :porcentaje_etc1
    :type cl:float
    :initform 0.0)
   (revoluciones2_etc1
    :reader revoluciones2_etc1
    :initarg :revoluciones2_etc1
    :type cl:float
    :initform 0.0)
   (revoluciones_eec1
    :reader revoluciones_eec1
    :initarg :revoluciones_eec1
    :type cl:float
    :initform 0.0)
   (porcentaje_eec1
    :reader porcentaje_eec1
    :initarg :porcentaje_eec1
    :type cl:float
    :initform 0.0)
   (marcha
    :reader marcha
    :initarg :marcha
    :type cl:float
    :initform 0.0)
   (marcha_2
    :reader marcha_2
    :initarg :marcha_2
    :type cl:float
    :initform 0.0)
   (marcha_3
    :reader marcha_3
    :initarg :marcha_3
    :type cl:float
    :initform 0.0)
   (opcion_ebc1
    :reader opcion_ebc1
    :initarg :opcion_ebc1
    :type cl:string
    :initform "")
   (porcentaje_ebc1
    :reader porcentaje_ebc1
    :initarg :porcentaje_ebc1
    :type cl:float
    :initform 0.0)
   (opcion_ccvs
    :reader opcion_ccvs
    :initarg :opcion_ccvs
    :type cl:string
    :initform "")
   (km_h_ccvs
    :reader km_h_ccvs
    :initarg :km_h_ccvs
    :type cl:float
    :initform 0.0)
   (opcion2_ccvs
    :reader opcion2_ccvs
    :initarg :opcion2_ccvs
    :type cl:string
    :initform "")
   (opcion3_ccvs
    :reader opcion3_ccvs
    :initarg :opcion3_ccvs
    :type cl:string
    :initform "")
   (opcion4_ccvs
    :reader opcion4_ccvs
    :initarg :opcion4_ccvs
    :type cl:string
    :initform "")
   (pto_state_ccvs
    :reader pto_state_ccvs
    :initarg :pto_state_ccvs
    :type cl:string
    :initform "")
   (opcion_aux
    :reader opcion_aux
    :initarg :opcion_aux
    :type cl:string
    :initform "")
   (opcion2_aux
    :reader opcion2_aux
    :initarg :opcion2_aux
    :type cl:string
    :initform "")
   (opcion3_aux
    :reader opcion3_aux
    :initarg :opcion3_aux
    :type cl:string
    :initform "")
   (opcion_eec2
    :reader opcion_eec2
    :initarg :opcion_eec2
    :type cl:string
    :initform "")
   (opcion2_eec2
    :reader opcion2_eec2
    :initarg :opcion2_eec2
    :type cl:string
    :initform "")
   (porcentaje_eec2
    :reader porcentaje_eec2
    :initarg :porcentaje_eec2
    :type cl:float
    :initform 0.0)
   (porcentaje2_eec2
    :reader porcentaje2_eec2
    :initarg :porcentaje2_eec2
    :type cl:float
    :initform 0.0)
   (grados_et
    :reader grados_et
    :initarg :grados_et
    :type cl:float
    :initform 0.0)
   (grados2_et
    :reader grados2_et
    :initarg :grados2_et
    :type cl:float
    :initform 0.0)
   (grados3_et
    :reader grados3_et
    :initarg :grados3_et
    :type cl:float
    :initform 0.0)
   (bares_ef
    :reader bares_ef
    :initarg :bares_ef
    :type cl:float
    :initform 0.0)
   (bares_sp
    :reader bares_sp
    :initarg :bares_sp
    :type cl:float
    :initform 0.0)
   (bares2_sp
    :reader bares2_sp
    :initarg :bares2_sp
    :type cl:float
    :initform 0.0)
   (bares3_sp
    :reader bares3_sp
    :initarg :bares3_sp
    :type cl:float
    :initform 0.0)
   (bares4_sp
    :reader bares4_sp
    :initarg :bares4_sp
    :type cl:float
    :initform 0.0)
   (bares5_sp
    :reader bares5_sp
    :initarg :bares5_sp
    :type cl:float
    :initform 0.0)
   (bares6_sp
    :reader bares6_sp
    :initarg :bares6_sp
    :type cl:float
    :initform 0.0)
   (bares_ac
    :reader bares_ac
    :initarg :bares_ac
    :type cl:float
    :initform 0.0)
   (grados_ac
    :reader grados_ac
    :initarg :grados_ac
    :type cl:float
    :initform 0.0)
   (kilometros_vdhr
    :reader kilometros_vdhr
    :initarg :kilometros_vdhr
    :type cl:float
    :initform 0.0)
   (axle
    :reader axle
    :initarg :axle
    :type cl:string
    :initform "")
   (kilogramos_vheacs
    :reader kilogramos_vheacs
    :initarg :kilogramos_vheacs
    :type cl:float
    :initform 0.0)
   (horas
    :reader horas
    :initarg :horas
    :type cl:float
    :initform 0.0)
   (driverWorking1
    :reader driverWorking1
    :initarg :driverWorking1
    :type cl:string
    :initform "")
   (driverWorking2
    :reader driverWorking2
    :initarg :driverWorking2
    :type cl:string
    :initform "")
   (opcion_t
    :reader opcion_t
    :initarg :opcion_t
    :type cl:string
    :initform "")
   (driverTime1
    :reader driverTime1
    :initarg :driverTime1
    :type cl:string
    :initform "")
   (opcion2_t
    :reader opcion2_t
    :initarg :opcion2_t
    :type cl:string
    :initform "")
   (opcion3_t
    :reader opcion3_t
    :initarg :opcion3_t
    :type cl:string
    :initform "")
   (driverTime2
    :reader driverTime2
    :initarg :driverTime2
    :type cl:string
    :initform "")
   (opcion4_t
    :reader opcion4_t
    :initarg :opcion4_t
    :type cl:string
    :initform "")
   (opcion5_t
    :reader opcion5_t
    :initarg :opcion5_t
    :type cl:string
    :initform "")
   (opcion6_t
    :reader opcion6_t
    :initarg :opcion6_t
    :type cl:string
    :initform "")
   (opcion7_t
    :reader opcion7_t
    :initarg :opcion7_t
    :type cl:string
    :initform "")
   (km_h_t
    :reader km_h_t
    :initarg :km_h_t
    :type cl:float
    :initform 0.0)
   (porcentaje_erc
    :reader porcentaje_erc
    :initarg :porcentaje_erc
    :type cl:float
    :initform 0.0)
   (opcion_auxstat
    :reader opcion_auxstat
    :initarg :opcion_auxstat
    :type cl:string
    :initform "")
   (opcion2_auxstat
    :reader opcion2_auxstat
    :initarg :opcion2_auxstat
    :type cl:string
    :initform "")
   (litros
    :reader litros
    :initarg :litros
    :type cl:float
    :initform 0.0)
   (km_l
    :reader km_l
    :initarg :km_l
    :type cl:float
    :initform 0.0)
   (opcion_etc3
    :reader opcion_etc3
    :initarg :opcion_etc3
    :type cl:string
    :initform "")
   (opcion2_etc3
    :reader opcion2_etc3
    :initarg :opcion2_etc3
    :type cl:string
    :initform "")
   (pto1state
    :reader pto1state
    :initarg :pto1state
    :type cl:string
    :initform "")
   (pto2state
    :reader pto2state
    :initarg :pto2state
    :type cl:string
    :initform "")
   (nmvstate
    :reader nmvstate
    :initarg :nmvstate
    :type cl:string
    :initform ""))
)

(cl:defclass messageCAN (<messageCAN>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <messageCAN>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'messageCAN)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Modulo_Conduccion-msg:<messageCAN> is deprecated: use Modulo_Conduccion-msg:messageCAN instead.")))

(cl:ensure-generic-function 'senal-val :lambda-list '(m))
(cl:defmethod senal-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:senal-val is deprecated.  Use Modulo_Conduccion-msg:senal instead.")
  (senal m))

(cl:ensure-generic-function 'opcion_etc1-val :lambda-list '(m))
(cl:defmethod opcion_etc1-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion_etc1-val is deprecated.  Use Modulo_Conduccion-msg:opcion_etc1 instead.")
  (opcion_etc1 m))

(cl:ensure-generic-function 'revoluciones_etc1-val :lambda-list '(m))
(cl:defmethod revoluciones_etc1-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:revoluciones_etc1-val is deprecated.  Use Modulo_Conduccion-msg:revoluciones_etc1 instead.")
  (revoluciones_etc1 m))

(cl:ensure-generic-function 'porcentaje_etc1-val :lambda-list '(m))
(cl:defmethod porcentaje_etc1-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:porcentaje_etc1-val is deprecated.  Use Modulo_Conduccion-msg:porcentaje_etc1 instead.")
  (porcentaje_etc1 m))

(cl:ensure-generic-function 'revoluciones2_etc1-val :lambda-list '(m))
(cl:defmethod revoluciones2_etc1-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:revoluciones2_etc1-val is deprecated.  Use Modulo_Conduccion-msg:revoluciones2_etc1 instead.")
  (revoluciones2_etc1 m))

(cl:ensure-generic-function 'revoluciones_eec1-val :lambda-list '(m))
(cl:defmethod revoluciones_eec1-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:revoluciones_eec1-val is deprecated.  Use Modulo_Conduccion-msg:revoluciones_eec1 instead.")
  (revoluciones_eec1 m))

(cl:ensure-generic-function 'porcentaje_eec1-val :lambda-list '(m))
(cl:defmethod porcentaje_eec1-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:porcentaje_eec1-val is deprecated.  Use Modulo_Conduccion-msg:porcentaje_eec1 instead.")
  (porcentaje_eec1 m))

(cl:ensure-generic-function 'marcha-val :lambda-list '(m))
(cl:defmethod marcha-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:marcha-val is deprecated.  Use Modulo_Conduccion-msg:marcha instead.")
  (marcha m))

(cl:ensure-generic-function 'marcha_2-val :lambda-list '(m))
(cl:defmethod marcha_2-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:marcha_2-val is deprecated.  Use Modulo_Conduccion-msg:marcha_2 instead.")
  (marcha_2 m))

(cl:ensure-generic-function 'marcha_3-val :lambda-list '(m))
(cl:defmethod marcha_3-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:marcha_3-val is deprecated.  Use Modulo_Conduccion-msg:marcha_3 instead.")
  (marcha_3 m))

(cl:ensure-generic-function 'opcion_ebc1-val :lambda-list '(m))
(cl:defmethod opcion_ebc1-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion_ebc1-val is deprecated.  Use Modulo_Conduccion-msg:opcion_ebc1 instead.")
  (opcion_ebc1 m))

(cl:ensure-generic-function 'porcentaje_ebc1-val :lambda-list '(m))
(cl:defmethod porcentaje_ebc1-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:porcentaje_ebc1-val is deprecated.  Use Modulo_Conduccion-msg:porcentaje_ebc1 instead.")
  (porcentaje_ebc1 m))

(cl:ensure-generic-function 'opcion_ccvs-val :lambda-list '(m))
(cl:defmethod opcion_ccvs-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion_ccvs-val is deprecated.  Use Modulo_Conduccion-msg:opcion_ccvs instead.")
  (opcion_ccvs m))

(cl:ensure-generic-function 'km_h_ccvs-val :lambda-list '(m))
(cl:defmethod km_h_ccvs-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:km_h_ccvs-val is deprecated.  Use Modulo_Conduccion-msg:km_h_ccvs instead.")
  (km_h_ccvs m))

(cl:ensure-generic-function 'opcion2_ccvs-val :lambda-list '(m))
(cl:defmethod opcion2_ccvs-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion2_ccvs-val is deprecated.  Use Modulo_Conduccion-msg:opcion2_ccvs instead.")
  (opcion2_ccvs m))

(cl:ensure-generic-function 'opcion3_ccvs-val :lambda-list '(m))
(cl:defmethod opcion3_ccvs-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion3_ccvs-val is deprecated.  Use Modulo_Conduccion-msg:opcion3_ccvs instead.")
  (opcion3_ccvs m))

(cl:ensure-generic-function 'opcion4_ccvs-val :lambda-list '(m))
(cl:defmethod opcion4_ccvs-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion4_ccvs-val is deprecated.  Use Modulo_Conduccion-msg:opcion4_ccvs instead.")
  (opcion4_ccvs m))

(cl:ensure-generic-function 'pto_state_ccvs-val :lambda-list '(m))
(cl:defmethod pto_state_ccvs-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:pto_state_ccvs-val is deprecated.  Use Modulo_Conduccion-msg:pto_state_ccvs instead.")
  (pto_state_ccvs m))

(cl:ensure-generic-function 'opcion_aux-val :lambda-list '(m))
(cl:defmethod opcion_aux-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion_aux-val is deprecated.  Use Modulo_Conduccion-msg:opcion_aux instead.")
  (opcion_aux m))

(cl:ensure-generic-function 'opcion2_aux-val :lambda-list '(m))
(cl:defmethod opcion2_aux-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion2_aux-val is deprecated.  Use Modulo_Conduccion-msg:opcion2_aux instead.")
  (opcion2_aux m))

(cl:ensure-generic-function 'opcion3_aux-val :lambda-list '(m))
(cl:defmethod opcion3_aux-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion3_aux-val is deprecated.  Use Modulo_Conduccion-msg:opcion3_aux instead.")
  (opcion3_aux m))

(cl:ensure-generic-function 'opcion_eec2-val :lambda-list '(m))
(cl:defmethod opcion_eec2-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion_eec2-val is deprecated.  Use Modulo_Conduccion-msg:opcion_eec2 instead.")
  (opcion_eec2 m))

(cl:ensure-generic-function 'opcion2_eec2-val :lambda-list '(m))
(cl:defmethod opcion2_eec2-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion2_eec2-val is deprecated.  Use Modulo_Conduccion-msg:opcion2_eec2 instead.")
  (opcion2_eec2 m))

(cl:ensure-generic-function 'porcentaje_eec2-val :lambda-list '(m))
(cl:defmethod porcentaje_eec2-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:porcentaje_eec2-val is deprecated.  Use Modulo_Conduccion-msg:porcentaje_eec2 instead.")
  (porcentaje_eec2 m))

(cl:ensure-generic-function 'porcentaje2_eec2-val :lambda-list '(m))
(cl:defmethod porcentaje2_eec2-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:porcentaje2_eec2-val is deprecated.  Use Modulo_Conduccion-msg:porcentaje2_eec2 instead.")
  (porcentaje2_eec2 m))

(cl:ensure-generic-function 'grados_et-val :lambda-list '(m))
(cl:defmethod grados_et-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:grados_et-val is deprecated.  Use Modulo_Conduccion-msg:grados_et instead.")
  (grados_et m))

(cl:ensure-generic-function 'grados2_et-val :lambda-list '(m))
(cl:defmethod grados2_et-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:grados2_et-val is deprecated.  Use Modulo_Conduccion-msg:grados2_et instead.")
  (grados2_et m))

(cl:ensure-generic-function 'grados3_et-val :lambda-list '(m))
(cl:defmethod grados3_et-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:grados3_et-val is deprecated.  Use Modulo_Conduccion-msg:grados3_et instead.")
  (grados3_et m))

(cl:ensure-generic-function 'bares_ef-val :lambda-list '(m))
(cl:defmethod bares_ef-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:bares_ef-val is deprecated.  Use Modulo_Conduccion-msg:bares_ef instead.")
  (bares_ef m))

(cl:ensure-generic-function 'bares_sp-val :lambda-list '(m))
(cl:defmethod bares_sp-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:bares_sp-val is deprecated.  Use Modulo_Conduccion-msg:bares_sp instead.")
  (bares_sp m))

(cl:ensure-generic-function 'bares2_sp-val :lambda-list '(m))
(cl:defmethod bares2_sp-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:bares2_sp-val is deprecated.  Use Modulo_Conduccion-msg:bares2_sp instead.")
  (bares2_sp m))

(cl:ensure-generic-function 'bares3_sp-val :lambda-list '(m))
(cl:defmethod bares3_sp-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:bares3_sp-val is deprecated.  Use Modulo_Conduccion-msg:bares3_sp instead.")
  (bares3_sp m))

(cl:ensure-generic-function 'bares4_sp-val :lambda-list '(m))
(cl:defmethod bares4_sp-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:bares4_sp-val is deprecated.  Use Modulo_Conduccion-msg:bares4_sp instead.")
  (bares4_sp m))

(cl:ensure-generic-function 'bares5_sp-val :lambda-list '(m))
(cl:defmethod bares5_sp-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:bares5_sp-val is deprecated.  Use Modulo_Conduccion-msg:bares5_sp instead.")
  (bares5_sp m))

(cl:ensure-generic-function 'bares6_sp-val :lambda-list '(m))
(cl:defmethod bares6_sp-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:bares6_sp-val is deprecated.  Use Modulo_Conduccion-msg:bares6_sp instead.")
  (bares6_sp m))

(cl:ensure-generic-function 'bares_ac-val :lambda-list '(m))
(cl:defmethod bares_ac-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:bares_ac-val is deprecated.  Use Modulo_Conduccion-msg:bares_ac instead.")
  (bares_ac m))

(cl:ensure-generic-function 'grados_ac-val :lambda-list '(m))
(cl:defmethod grados_ac-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:grados_ac-val is deprecated.  Use Modulo_Conduccion-msg:grados_ac instead.")
  (grados_ac m))

(cl:ensure-generic-function 'kilometros_vdhr-val :lambda-list '(m))
(cl:defmethod kilometros_vdhr-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:kilometros_vdhr-val is deprecated.  Use Modulo_Conduccion-msg:kilometros_vdhr instead.")
  (kilometros_vdhr m))

(cl:ensure-generic-function 'axle-val :lambda-list '(m))
(cl:defmethod axle-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:axle-val is deprecated.  Use Modulo_Conduccion-msg:axle instead.")
  (axle m))

(cl:ensure-generic-function 'kilogramos_vheacs-val :lambda-list '(m))
(cl:defmethod kilogramos_vheacs-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:kilogramos_vheacs-val is deprecated.  Use Modulo_Conduccion-msg:kilogramos_vheacs instead.")
  (kilogramos_vheacs m))

(cl:ensure-generic-function 'horas-val :lambda-list '(m))
(cl:defmethod horas-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:horas-val is deprecated.  Use Modulo_Conduccion-msg:horas instead.")
  (horas m))

(cl:ensure-generic-function 'driverWorking1-val :lambda-list '(m))
(cl:defmethod driverWorking1-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:driverWorking1-val is deprecated.  Use Modulo_Conduccion-msg:driverWorking1 instead.")
  (driverWorking1 m))

(cl:ensure-generic-function 'driverWorking2-val :lambda-list '(m))
(cl:defmethod driverWorking2-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:driverWorking2-val is deprecated.  Use Modulo_Conduccion-msg:driverWorking2 instead.")
  (driverWorking2 m))

(cl:ensure-generic-function 'opcion_t-val :lambda-list '(m))
(cl:defmethod opcion_t-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion_t-val is deprecated.  Use Modulo_Conduccion-msg:opcion_t instead.")
  (opcion_t m))

(cl:ensure-generic-function 'driverTime1-val :lambda-list '(m))
(cl:defmethod driverTime1-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:driverTime1-val is deprecated.  Use Modulo_Conduccion-msg:driverTime1 instead.")
  (driverTime1 m))

(cl:ensure-generic-function 'opcion2_t-val :lambda-list '(m))
(cl:defmethod opcion2_t-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion2_t-val is deprecated.  Use Modulo_Conduccion-msg:opcion2_t instead.")
  (opcion2_t m))

(cl:ensure-generic-function 'opcion3_t-val :lambda-list '(m))
(cl:defmethod opcion3_t-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion3_t-val is deprecated.  Use Modulo_Conduccion-msg:opcion3_t instead.")
  (opcion3_t m))

(cl:ensure-generic-function 'driverTime2-val :lambda-list '(m))
(cl:defmethod driverTime2-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:driverTime2-val is deprecated.  Use Modulo_Conduccion-msg:driverTime2 instead.")
  (driverTime2 m))

(cl:ensure-generic-function 'opcion4_t-val :lambda-list '(m))
(cl:defmethod opcion4_t-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion4_t-val is deprecated.  Use Modulo_Conduccion-msg:opcion4_t instead.")
  (opcion4_t m))

(cl:ensure-generic-function 'opcion5_t-val :lambda-list '(m))
(cl:defmethod opcion5_t-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion5_t-val is deprecated.  Use Modulo_Conduccion-msg:opcion5_t instead.")
  (opcion5_t m))

(cl:ensure-generic-function 'opcion6_t-val :lambda-list '(m))
(cl:defmethod opcion6_t-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion6_t-val is deprecated.  Use Modulo_Conduccion-msg:opcion6_t instead.")
  (opcion6_t m))

(cl:ensure-generic-function 'opcion7_t-val :lambda-list '(m))
(cl:defmethod opcion7_t-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion7_t-val is deprecated.  Use Modulo_Conduccion-msg:opcion7_t instead.")
  (opcion7_t m))

(cl:ensure-generic-function 'km_h_t-val :lambda-list '(m))
(cl:defmethod km_h_t-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:km_h_t-val is deprecated.  Use Modulo_Conduccion-msg:km_h_t instead.")
  (km_h_t m))

(cl:ensure-generic-function 'porcentaje_erc-val :lambda-list '(m))
(cl:defmethod porcentaje_erc-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:porcentaje_erc-val is deprecated.  Use Modulo_Conduccion-msg:porcentaje_erc instead.")
  (porcentaje_erc m))

(cl:ensure-generic-function 'opcion_auxstat-val :lambda-list '(m))
(cl:defmethod opcion_auxstat-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion_auxstat-val is deprecated.  Use Modulo_Conduccion-msg:opcion_auxstat instead.")
  (opcion_auxstat m))

(cl:ensure-generic-function 'opcion2_auxstat-val :lambda-list '(m))
(cl:defmethod opcion2_auxstat-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion2_auxstat-val is deprecated.  Use Modulo_Conduccion-msg:opcion2_auxstat instead.")
  (opcion2_auxstat m))

(cl:ensure-generic-function 'litros-val :lambda-list '(m))
(cl:defmethod litros-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:litros-val is deprecated.  Use Modulo_Conduccion-msg:litros instead.")
  (litros m))

(cl:ensure-generic-function 'km_l-val :lambda-list '(m))
(cl:defmethod km_l-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:km_l-val is deprecated.  Use Modulo_Conduccion-msg:km_l instead.")
  (km_l m))

(cl:ensure-generic-function 'opcion_etc3-val :lambda-list '(m))
(cl:defmethod opcion_etc3-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion_etc3-val is deprecated.  Use Modulo_Conduccion-msg:opcion_etc3 instead.")
  (opcion_etc3 m))

(cl:ensure-generic-function 'opcion2_etc3-val :lambda-list '(m))
(cl:defmethod opcion2_etc3-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:opcion2_etc3-val is deprecated.  Use Modulo_Conduccion-msg:opcion2_etc3 instead.")
  (opcion2_etc3 m))

(cl:ensure-generic-function 'pto1state-val :lambda-list '(m))
(cl:defmethod pto1state-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:pto1state-val is deprecated.  Use Modulo_Conduccion-msg:pto1state instead.")
  (pto1state m))

(cl:ensure-generic-function 'pto2state-val :lambda-list '(m))
(cl:defmethod pto2state-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:pto2state-val is deprecated.  Use Modulo_Conduccion-msg:pto2state instead.")
  (pto2state m))

(cl:ensure-generic-function 'nmvstate-val :lambda-list '(m))
(cl:defmethod nmvstate-val ((m <messageCAN>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Modulo_Conduccion-msg:nmvstate-val is deprecated.  Use Modulo_Conduccion-msg:nmvstate instead.")
  (nmvstate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <messageCAN>) ostream)
  "Serializes a message object of type '<messageCAN>"
  (cl:let* ((signed (cl:slot-value msg 'senal)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion_etc1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion_etc1))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'revoluciones_etc1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'porcentaje_etc1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'revoluciones2_etc1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'revoluciones_eec1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'porcentaje_eec1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'marcha))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'marcha_2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'marcha_3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion_ebc1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion_ebc1))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'porcentaje_ebc1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion_ccvs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion_ccvs))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'km_h_ccvs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion2_ccvs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion2_ccvs))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion3_ccvs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion3_ccvs))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion4_ccvs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion4_ccvs))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pto_state_ccvs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pto_state_ccvs))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion_aux))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion_aux))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion2_aux))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion2_aux))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion3_aux))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion3_aux))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion_eec2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion_eec2))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion2_eec2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion2_eec2))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'porcentaje_eec2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'porcentaje2_eec2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'grados_et))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'grados2_et))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'grados3_et))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bares_ef))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bares_sp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bares2_sp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bares3_sp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bares4_sp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bares5_sp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bares6_sp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'bares_ac))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'grados_ac))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'kilometros_vdhr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'axle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'axle))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'kilogramos_vheacs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'horas))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'driverWorking1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'driverWorking1))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'driverWorking2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'driverWorking2))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion_t))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion_t))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'driverTime1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'driverTime1))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion2_t))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion2_t))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion3_t))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion3_t))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'driverTime2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'driverTime2))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion4_t))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion4_t))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion5_t))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion5_t))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion6_t))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion6_t))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion7_t))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion7_t))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'km_h_t))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'porcentaje_erc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion_auxstat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion_auxstat))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion2_auxstat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion2_auxstat))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'litros))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'km_l))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion_etc3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion_etc3))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'opcion2_etc3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'opcion2_etc3))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pto1state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pto1state))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pto2state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pto2state))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'nmvstate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'nmvstate))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <messageCAN>) istream)
  "Deserializes a message object of type '<messageCAN>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'senal) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion_etc1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion_etc1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'revoluciones_etc1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'porcentaje_etc1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'revoluciones2_etc1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'revoluciones_eec1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'porcentaje_eec1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'marcha) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'marcha_2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'marcha_3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion_ebc1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion_ebc1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'porcentaje_ebc1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion_ccvs) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion_ccvs) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'km_h_ccvs) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion2_ccvs) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion2_ccvs) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion3_ccvs) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion3_ccvs) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion4_ccvs) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion4_ccvs) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pto_state_ccvs) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pto_state_ccvs) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion_aux) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion_aux) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion2_aux) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion2_aux) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion3_aux) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion3_aux) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion_eec2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion_eec2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion2_eec2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion2_eec2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'porcentaje_eec2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'porcentaje2_eec2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grados_et) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grados2_et) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grados3_et) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bares_ef) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bares_sp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bares2_sp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bares3_sp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bares4_sp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bares5_sp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bares6_sp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bares_ac) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grados_ac) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kilometros_vdhr) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'axle) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'axle) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'kilogramos_vheacs) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'horas) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'driverWorking1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'driverWorking1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'driverWorking2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'driverWorking2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion_t) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion_t) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'driverTime1) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'driverTime1) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion2_t) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion2_t) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion3_t) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion3_t) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'driverTime2) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'driverTime2) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion4_t) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion4_t) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion5_t) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion5_t) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion6_t) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion6_t) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion7_t) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion7_t) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'km_h_t) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'porcentaje_erc) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion_auxstat) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion_auxstat) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion2_auxstat) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion2_auxstat) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'litros) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'km_l) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion_etc3) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion_etc3) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'opcion2_etc3) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'opcion2_etc3) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pto1state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pto1state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pto2state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pto2state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nmvstate) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'nmvstate) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<messageCAN>)))
  "Returns string type for a message object of type '<messageCAN>"
  "Modulo_Conduccion/messageCAN")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'messageCAN)))
  "Returns string type for a message object of type 'messageCAN"
  "Modulo_Conduccion/messageCAN")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<messageCAN>)))
  "Returns md5sum for a message object of type '<messageCAN>"
  "02717f479fdfd6d478732dcace63ec89")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'messageCAN)))
  "Returns md5sum for a message object of type 'messageCAN"
  "02717f479fdfd6d478732dcace63ec89")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<messageCAN>)))
  "Returns full string definition for message of type '<messageCAN>"
  (cl:format cl:nil "# Single scan from a Bus CAN~%~%int8 senal~%~%~%# Señal 1 - ELECTRONIC TRANSMISSION CONTROLLER1~%string opcion_etc1		#string~%float64 revoluciones_etc1	#double~%float64 porcentaje_etc1		#double~%float64 revoluciones2_etc1	#double~%~%~%# Señal 2 - ELECTRONIC ENGINE CONTROLLER1~%float64 revoluciones_eec1	#double~%float64 porcentaje_eec1		#double (entero)~%~%~%# Señal 3 - ELECTRONIC TRANSMISSION CONTROLLER2~%float64 marcha			#double (entero)~%float64 marcha_2		#double~%float64 marcha_3		#double (entero)~%~%~%# Señal 4 - ELECTRONIC BRAKE CONTROLLER1~%string opcion_ebc1		#string~%float64 porcentaje_ebc1		#double~%~%~%# Señal 5 - CRUISE CONTROL VEHICULE SPEED~%string opcion_ccvs		#string~%float64 km_h_ccvs		#double~%string opcion2_ccvs		#string~%string opcion3_ccvs		#string~%string opcion4_ccvs		#string~%string pto_state_ccvs		#string~%~%~%# Señal 6 - AUXILIARY STATE~%string opcion_aux		#string~%string opcion2_aux		#string~%string opcion3_aux		#string~%~%~%# Señal 7 - ELECTRONIC ENGINE CONTROLLER2~%string opcion_eec2		#string~%string opcion2_eec2		#string~%float64 porcentaje_eec2		#double~%float64 porcentaje2_eec2	#double~%~%~%# Señal 8 - ENGINE TEMPERATURE~%float64 grados_et		#double (entero)~%float64 grados2_et		#double (entero)~%float64 grados3_et		#double~%~%~%# Señal 9 - ENGINE FLUID~%float64 bares_ef		#double~%~%~%# Señal 10 - SUPPLY PRESSURE ~%float64 bares_sp		#double~%float64 bares2_sp		#double~%float64 bares3_sp		#double~%float64 bares4_sp		#double~%float64 bares5_sp		#double~%float64 bares6_sp		#double~%~%~%# Señal 11 - AMBIENT CONDITIONS~%float64 bares_ac		#double~%float64 grados_ac		#double~%~%~%# Señal 12 - VEHICULE DISTANCE HIGH RESOLUTION~%float64 kilometros_vdhr		#double~%~%~%# Señal 13 - VEHICULE WEIGHT~%string axle			#string~%float64 kilogramos_vheacs	#double~%~%~%# Señal 14 - ENGINE HOURS~%float64 horas			#double~%~%~%# Señal 15 - TACOGRAPH~%string driverWorking1		#string~%string driverWorking2		#string~%string opcion_t			#string~%string driverTime1		#string~%string opcion2_t		#string~%string opcion3_t		#string~%string driverTime2		#string~%string opcion4_t		#string~%string opcion5_t		#string~%string opcion6_t		#string~%string opcion7_t		#string~%float64 km_h_t			#double~%~%~%# Señal 16 - ELECTRONIC RETARDER CONTROLLER EXHAUST~%float64 porcentaje_erc		#double (entero)~%~%~%# Señal 17 - AUX STAT~%string opcion_auxstat		#string~%string opcion2_auxstat		#string~%~%~%# Señal 18 - FUEL ECONOMY~%float64 litros			#double~%float64 km_l			#double~%~%~%# Señal 19 - ELECTRONIC TRANSMISSION CONTROLLER3 ~%string opcion_etc3		#string~%string opcion2_etc3		#string~%string pto1state		#string~%string pto2state		#string~%string nmvstate			#string~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'messageCAN)))
  "Returns full string definition for message of type 'messageCAN"
  (cl:format cl:nil "# Single scan from a Bus CAN~%~%int8 senal~%~%~%# Señal 1 - ELECTRONIC TRANSMISSION CONTROLLER1~%string opcion_etc1		#string~%float64 revoluciones_etc1	#double~%float64 porcentaje_etc1		#double~%float64 revoluciones2_etc1	#double~%~%~%# Señal 2 - ELECTRONIC ENGINE CONTROLLER1~%float64 revoluciones_eec1	#double~%float64 porcentaje_eec1		#double (entero)~%~%~%# Señal 3 - ELECTRONIC TRANSMISSION CONTROLLER2~%float64 marcha			#double (entero)~%float64 marcha_2		#double~%float64 marcha_3		#double (entero)~%~%~%# Señal 4 - ELECTRONIC BRAKE CONTROLLER1~%string opcion_ebc1		#string~%float64 porcentaje_ebc1		#double~%~%~%# Señal 5 - CRUISE CONTROL VEHICULE SPEED~%string opcion_ccvs		#string~%float64 km_h_ccvs		#double~%string opcion2_ccvs		#string~%string opcion3_ccvs		#string~%string opcion4_ccvs		#string~%string pto_state_ccvs		#string~%~%~%# Señal 6 - AUXILIARY STATE~%string opcion_aux		#string~%string opcion2_aux		#string~%string opcion3_aux		#string~%~%~%# Señal 7 - ELECTRONIC ENGINE CONTROLLER2~%string opcion_eec2		#string~%string opcion2_eec2		#string~%float64 porcentaje_eec2		#double~%float64 porcentaje2_eec2	#double~%~%~%# Señal 8 - ENGINE TEMPERATURE~%float64 grados_et		#double (entero)~%float64 grados2_et		#double (entero)~%float64 grados3_et		#double~%~%~%# Señal 9 - ENGINE FLUID~%float64 bares_ef		#double~%~%~%# Señal 10 - SUPPLY PRESSURE ~%float64 bares_sp		#double~%float64 bares2_sp		#double~%float64 bares3_sp		#double~%float64 bares4_sp		#double~%float64 bares5_sp		#double~%float64 bares6_sp		#double~%~%~%# Señal 11 - AMBIENT CONDITIONS~%float64 bares_ac		#double~%float64 grados_ac		#double~%~%~%# Señal 12 - VEHICULE DISTANCE HIGH RESOLUTION~%float64 kilometros_vdhr		#double~%~%~%# Señal 13 - VEHICULE WEIGHT~%string axle			#string~%float64 kilogramos_vheacs	#double~%~%~%# Señal 14 - ENGINE HOURS~%float64 horas			#double~%~%~%# Señal 15 - TACOGRAPH~%string driverWorking1		#string~%string driverWorking2		#string~%string opcion_t			#string~%string driverTime1		#string~%string opcion2_t		#string~%string opcion3_t		#string~%string driverTime2		#string~%string opcion4_t		#string~%string opcion5_t		#string~%string opcion6_t		#string~%string opcion7_t		#string~%float64 km_h_t			#double~%~%~%# Señal 16 - ELECTRONIC RETARDER CONTROLLER EXHAUST~%float64 porcentaje_erc		#double (entero)~%~%~%# Señal 17 - AUX STAT~%string opcion_auxstat		#string~%string opcion2_auxstat		#string~%~%~%# Señal 18 - FUEL ECONOMY~%float64 litros			#double~%float64 km_l			#double~%~%~%# Señal 19 - ELECTRONIC TRANSMISSION CONTROLLER3 ~%string opcion_etc3		#string~%string opcion2_etc3		#string~%string pto1state		#string~%string pto2state		#string~%string nmvstate			#string~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <messageCAN>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'opcion_etc1))
     8
     8
     8
     8
     8
     8
     8
     8
     4 (cl:length (cl:slot-value msg 'opcion_ebc1))
     8
     4 (cl:length (cl:slot-value msg 'opcion_ccvs))
     8
     4 (cl:length (cl:slot-value msg 'opcion2_ccvs))
     4 (cl:length (cl:slot-value msg 'opcion3_ccvs))
     4 (cl:length (cl:slot-value msg 'opcion4_ccvs))
     4 (cl:length (cl:slot-value msg 'pto_state_ccvs))
     4 (cl:length (cl:slot-value msg 'opcion_aux))
     4 (cl:length (cl:slot-value msg 'opcion2_aux))
     4 (cl:length (cl:slot-value msg 'opcion3_aux))
     4 (cl:length (cl:slot-value msg 'opcion_eec2))
     4 (cl:length (cl:slot-value msg 'opcion2_eec2))
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     4 (cl:length (cl:slot-value msg 'axle))
     8
     8
     4 (cl:length (cl:slot-value msg 'driverWorking1))
     4 (cl:length (cl:slot-value msg 'driverWorking2))
     4 (cl:length (cl:slot-value msg 'opcion_t))
     4 (cl:length (cl:slot-value msg 'driverTime1))
     4 (cl:length (cl:slot-value msg 'opcion2_t))
     4 (cl:length (cl:slot-value msg 'opcion3_t))
     4 (cl:length (cl:slot-value msg 'driverTime2))
     4 (cl:length (cl:slot-value msg 'opcion4_t))
     4 (cl:length (cl:slot-value msg 'opcion5_t))
     4 (cl:length (cl:slot-value msg 'opcion6_t))
     4 (cl:length (cl:slot-value msg 'opcion7_t))
     8
     8
     4 (cl:length (cl:slot-value msg 'opcion_auxstat))
     4 (cl:length (cl:slot-value msg 'opcion2_auxstat))
     8
     8
     4 (cl:length (cl:slot-value msg 'opcion_etc3))
     4 (cl:length (cl:slot-value msg 'opcion2_etc3))
     4 (cl:length (cl:slot-value msg 'pto1state))
     4 (cl:length (cl:slot-value msg 'pto2state))
     4 (cl:length (cl:slot-value msg 'nmvstate))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <messageCAN>))
  "Converts a ROS message object to a list"
  (cl:list 'messageCAN
    (cl:cons ':senal (senal msg))
    (cl:cons ':opcion_etc1 (opcion_etc1 msg))
    (cl:cons ':revoluciones_etc1 (revoluciones_etc1 msg))
    (cl:cons ':porcentaje_etc1 (porcentaje_etc1 msg))
    (cl:cons ':revoluciones2_etc1 (revoluciones2_etc1 msg))
    (cl:cons ':revoluciones_eec1 (revoluciones_eec1 msg))
    (cl:cons ':porcentaje_eec1 (porcentaje_eec1 msg))
    (cl:cons ':marcha (marcha msg))
    (cl:cons ':marcha_2 (marcha_2 msg))
    (cl:cons ':marcha_3 (marcha_3 msg))
    (cl:cons ':opcion_ebc1 (opcion_ebc1 msg))
    (cl:cons ':porcentaje_ebc1 (porcentaje_ebc1 msg))
    (cl:cons ':opcion_ccvs (opcion_ccvs msg))
    (cl:cons ':km_h_ccvs (km_h_ccvs msg))
    (cl:cons ':opcion2_ccvs (opcion2_ccvs msg))
    (cl:cons ':opcion3_ccvs (opcion3_ccvs msg))
    (cl:cons ':opcion4_ccvs (opcion4_ccvs msg))
    (cl:cons ':pto_state_ccvs (pto_state_ccvs msg))
    (cl:cons ':opcion_aux (opcion_aux msg))
    (cl:cons ':opcion2_aux (opcion2_aux msg))
    (cl:cons ':opcion3_aux (opcion3_aux msg))
    (cl:cons ':opcion_eec2 (opcion_eec2 msg))
    (cl:cons ':opcion2_eec2 (opcion2_eec2 msg))
    (cl:cons ':porcentaje_eec2 (porcentaje_eec2 msg))
    (cl:cons ':porcentaje2_eec2 (porcentaje2_eec2 msg))
    (cl:cons ':grados_et (grados_et msg))
    (cl:cons ':grados2_et (grados2_et msg))
    (cl:cons ':grados3_et (grados3_et msg))
    (cl:cons ':bares_ef (bares_ef msg))
    (cl:cons ':bares_sp (bares_sp msg))
    (cl:cons ':bares2_sp (bares2_sp msg))
    (cl:cons ':bares3_sp (bares3_sp msg))
    (cl:cons ':bares4_sp (bares4_sp msg))
    (cl:cons ':bares5_sp (bares5_sp msg))
    (cl:cons ':bares6_sp (bares6_sp msg))
    (cl:cons ':bares_ac (bares_ac msg))
    (cl:cons ':grados_ac (grados_ac msg))
    (cl:cons ':kilometros_vdhr (kilometros_vdhr msg))
    (cl:cons ':axle (axle msg))
    (cl:cons ':kilogramos_vheacs (kilogramos_vheacs msg))
    (cl:cons ':horas (horas msg))
    (cl:cons ':driverWorking1 (driverWorking1 msg))
    (cl:cons ':driverWorking2 (driverWorking2 msg))
    (cl:cons ':opcion_t (opcion_t msg))
    (cl:cons ':driverTime1 (driverTime1 msg))
    (cl:cons ':opcion2_t (opcion2_t msg))
    (cl:cons ':opcion3_t (opcion3_t msg))
    (cl:cons ':driverTime2 (driverTime2 msg))
    (cl:cons ':opcion4_t (opcion4_t msg))
    (cl:cons ':opcion5_t (opcion5_t msg))
    (cl:cons ':opcion6_t (opcion6_t msg))
    (cl:cons ':opcion7_t (opcion7_t msg))
    (cl:cons ':km_h_t (km_h_t msg))
    (cl:cons ':porcentaje_erc (porcentaje_erc msg))
    (cl:cons ':opcion_auxstat (opcion_auxstat msg))
    (cl:cons ':opcion2_auxstat (opcion2_auxstat msg))
    (cl:cons ':litros (litros msg))
    (cl:cons ':km_l (km_l msg))
    (cl:cons ':opcion_etc3 (opcion_etc3 msg))
    (cl:cons ':opcion2_etc3 (opcion2_etc3 msg))
    (cl:cons ':pto1state (pto1state msg))
    (cl:cons ':pto2state (pto2state msg))
    (cl:cons ':nmvstate (nmvstate msg))
))
