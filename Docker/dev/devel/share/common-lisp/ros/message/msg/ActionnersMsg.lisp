; Auto-generated. Do not edit!


(cl:in-package message-msg)


;//! \htmlinclude ActionnersMsg.msg.html

(cl:defclass <ActionnersMsg> (roslisp-msg-protocol:ros-message)
  ((act
    :reader act
    :initarg :act
    :type cl:string
    :initform "")
   (fail
    :reader fail
    :initarg :fail
    :type cl:boolean
    :initform cl:nil)
   (end
    :reader end
    :initarg :end
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ActionnersMsg (<ActionnersMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ActionnersMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ActionnersMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name message-msg:<ActionnersMsg> is deprecated: use message-msg:ActionnersMsg instead.")))

(cl:ensure-generic-function 'act-val :lambda-list '(m))
(cl:defmethod act-val ((m <ActionnersMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-msg:act-val is deprecated.  Use message-msg:act instead.")
  (act m))

(cl:ensure-generic-function 'fail-val :lambda-list '(m))
(cl:defmethod fail-val ((m <ActionnersMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-msg:fail-val is deprecated.  Use message-msg:fail instead.")
  (fail m))

(cl:ensure-generic-function 'end-val :lambda-list '(m))
(cl:defmethod end-val ((m <ActionnersMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-msg:end-val is deprecated.  Use message-msg:end instead.")
  (end m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ActionnersMsg>) ostream)
  "Serializes a message object of type '<ActionnersMsg>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'act))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'act))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fail) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'end) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ActionnersMsg>) istream)
  "Deserializes a message object of type '<ActionnersMsg>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'act) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'act) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'fail) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'end) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ActionnersMsg>)))
  "Returns string type for a message object of type '<ActionnersMsg>"
  "message/ActionnersMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ActionnersMsg)))
  "Returns string type for a message object of type 'ActionnersMsg"
  "message/ActionnersMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ActionnersMsg>)))
  "Returns md5sum for a message object of type '<ActionnersMsg>"
  "3dd2e8d97f70009cf8e47cbf3e745186")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ActionnersMsg)))
  "Returns md5sum for a message object of type 'ActionnersMsg"
  "3dd2e8d97f70009cf8e47cbf3e745186")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ActionnersMsg>)))
  "Returns full string definition for message of type '<ActionnersMsg>"
  (cl:format cl:nil "string act~%bool fail~%bool end~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ActionnersMsg)))
  "Returns full string definition for message of type 'ActionnersMsg"
  (cl:format cl:nil "string act~%bool fail~%bool end~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ActionnersMsg>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'act))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ActionnersMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ActionnersMsg
    (cl:cons ':act (act msg))
    (cl:cons ':fail (fail msg))
    (cl:cons ':end (end msg))
))
