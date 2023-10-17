; Auto-generated. Do not edit!


(cl:in-package message-msg)


;//! \htmlinclude InfoMsg.msg.html

(cl:defclass <InfoMsg> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (otherspos
    :reader otherspos
    :initarg :otherspos
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (possact
    :reader possact
    :initarg :possact
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (timeleft
    :reader timeleft
    :initarg :timeleft
    :type cl:float
    :initform 0.0))
)

(cl:defclass InfoMsg (<InfoMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InfoMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InfoMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name message-msg:<InfoMsg> is deprecated: use message-msg:InfoMsg instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <InfoMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-msg:x-val is deprecated.  Use message-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <InfoMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-msg:y-val is deprecated.  Use message-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'otherspos-val :lambda-list '(m))
(cl:defmethod otherspos-val ((m <InfoMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-msg:otherspos-val is deprecated.  Use message-msg:otherspos instead.")
  (otherspos m))

(cl:ensure-generic-function 'possact-val :lambda-list '(m))
(cl:defmethod possact-val ((m <InfoMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-msg:possact-val is deprecated.  Use message-msg:possact instead.")
  (possact m))

(cl:ensure-generic-function 'timeleft-val :lambda-list '(m))
(cl:defmethod timeleft-val ((m <InfoMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-msg:timeleft-val is deprecated.  Use message-msg:timeleft instead.")
  (timeleft m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InfoMsg>) ostream)
  "Serializes a message object of type '<InfoMsg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'otherspos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'otherspos))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'possact))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'possact))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'timeleft))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InfoMsg>) istream)
  "Deserializes a message object of type '<InfoMsg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'otherspos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'otherspos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'possact) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'possact)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536)))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeleft) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InfoMsg>)))
  "Returns string type for a message object of type '<InfoMsg>"
  "message/InfoMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InfoMsg)))
  "Returns string type for a message object of type 'InfoMsg"
  "message/InfoMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InfoMsg>)))
  "Returns md5sum for a message object of type '<InfoMsg>"
  "74243b5379e1c0f977f139ace36782e4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InfoMsg)))
  "Returns md5sum for a message object of type 'InfoMsg"
  "74243b5379e1c0f977f139ace36782e4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InfoMsg>)))
  "Returns full string definition for message of type '<InfoMsg>"
  (cl:format cl:nil "float32 x~%float32 y~%float32[] otherspos~%int16[] possact~%float32 timeleft~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InfoMsg)))
  "Returns full string definition for message of type 'InfoMsg"
  (cl:format cl:nil "float32 x~%float32 y~%float32[] otherspos~%int16[] possact~%float32 timeleft~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InfoMsg>))
  (cl:+ 0
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'otherspos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'possact) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InfoMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'InfoMsg
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':otherspos (otherspos msg))
    (cl:cons ':possact (possact msg))
    (cl:cons ':timeleft (timeleft msg))
))
