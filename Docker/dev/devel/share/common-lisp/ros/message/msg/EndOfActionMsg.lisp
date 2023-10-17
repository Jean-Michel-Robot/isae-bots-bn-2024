; Auto-generated. Do not edit!


(cl:in-package message-msg)


;//! \htmlinclude EndOfActionMsg.msg.html

(cl:defclass <EndOfActionMsg> (roslisp-msg-protocol:ros-message)
  ((exit
    :reader exit
    :initarg :exit
    :type cl:fixnum
    :initform 0)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass EndOfActionMsg (<EndOfActionMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EndOfActionMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EndOfActionMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name message-msg:<EndOfActionMsg> is deprecated: use message-msg:EndOfActionMsg instead.")))

(cl:ensure-generic-function 'exit-val :lambda-list '(m))
(cl:defmethod exit-val ((m <EndOfActionMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-msg:exit-val is deprecated.  Use message-msg:exit instead.")
  (exit m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <EndOfActionMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader message-msg:reason-val is deprecated.  Use message-msg:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EndOfActionMsg>) ostream)
  "Serializes a message object of type '<EndOfActionMsg>"
  (cl:let* ((signed (cl:slot-value msg 'exit)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EndOfActionMsg>) istream)
  "Deserializes a message object of type '<EndOfActionMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'exit) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EndOfActionMsg>)))
  "Returns string type for a message object of type '<EndOfActionMsg>"
  "message/EndOfActionMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EndOfActionMsg)))
  "Returns string type for a message object of type 'EndOfActionMsg"
  "message/EndOfActionMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EndOfActionMsg>)))
  "Returns md5sum for a message object of type '<EndOfActionMsg>"
  "6620ca083d96e08d9ae896bc1472aad2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EndOfActionMsg)))
  "Returns md5sum for a message object of type 'EndOfActionMsg"
  "6620ca083d96e08d9ae896bc1472aad2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EndOfActionMsg>)))
  "Returns full string definition for message of type '<EndOfActionMsg>"
  (cl:format cl:nil "int16 exit~%string reason~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EndOfActionMsg)))
  "Returns full string definition for message of type 'EndOfActionMsg"
  (cl:format cl:nil "int16 exit~%string reason~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EndOfActionMsg>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EndOfActionMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'EndOfActionMsg
    (cl:cons ':exit (exit msg))
    (cl:cons ':reason (reason msg))
))
