;; Auto-generated. Do not edit!


(when (boundp 'message::InfoMsg)
  (if (not (find-package "MESSAGE"))
    (make-package "MESSAGE"))
  (shadow 'InfoMsg (find-package "MESSAGE")))
(unless (find-package "MESSAGE::INFOMSG")
  (make-package "MESSAGE::INFOMSG"))

(in-package "ROS")
;;//! \htmlinclude InfoMsg.msg.html


(defclass message::InfoMsg
  :super ros::object
  :slots (_x _y _otherspos _possact _timeleft ))

(defmethod message::InfoMsg
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:otherspos __otherspos) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:possact __possact) (make-array 0 :initial-element 0 :element-type :integer))
    ((:timeleft __timeleft) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _otherspos __otherspos)
   (setq _possact __possact)
   (setq _timeleft (float __timeleft))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:otherspos
   (&optional __otherspos)
   (if __otherspos (setq _otherspos __otherspos)) _otherspos)
  (:possact
   (&optional __possact)
   (if __possact (setq _possact __possact)) _possact)
  (:timeleft
   (&optional __timeleft)
   (if __timeleft (setq _timeleft __timeleft)) _timeleft)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32[] _otherspos
    (* 4    (length _otherspos)) 4
    ;; int16[] _possact
    (* 2    (length _possact)) 4
    ;; float32 _timeleft
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _otherspos
     (write-long (length _otherspos) s)
     (dotimes (i (length _otherspos))
       (sys::poke (elt _otherspos i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int16[] _possact
     (write-long (length _possact) s)
     (dotimes (i (length _possact))
       (write-word (elt _possact i) s)
       )
     ;; float32 _timeleft
       (sys::poke _timeleft (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _otherspos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _otherspos (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _otherspos i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int16[] _possact
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _possact (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _possact i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; float32 _timeleft
     (setq _timeleft (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get message::InfoMsg :md5sum-) "74243b5379e1c0f977f139ace36782e4")
(setf (get message::InfoMsg :datatype-) "message/InfoMsg")
(setf (get message::InfoMsg :definition-)
      "float32 x
float32 y
float32[] otherspos
int16[] possact
float32 timeleft
")



(provide :message/InfoMsg "74243b5379e1c0f977f139ace36782e4")


