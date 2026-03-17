; Auto-generated. Do not edit!


(cl:in-package servo_ros-msg)


;//! \htmlinclude ServoMove.msg.html

(cl:defclass <ServoMove> (roslisp-msg-protocol:ros-message)
  ((servo_id
    :reader servo_id
    :initarg :servo_id
    :type cl:fixnum
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:fixnum
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ServoMove (<ServoMove>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServoMove>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServoMove)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name servo_ros-msg:<ServoMove> is deprecated: use servo_ros-msg:ServoMove instead.")))

(cl:ensure-generic-function 'servo_id-val :lambda-list '(m))
(cl:defmethod servo_id-val ((m <ServoMove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_ros-msg:servo_id-val is deprecated.  Use servo_ros-msg:servo_id instead.")
  (servo_id m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <ServoMove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_ros-msg:angle-val is deprecated.  Use servo_ros-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <ServoMove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_ros-msg:time-val is deprecated.  Use servo_ros-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServoMove>) ostream)
  "Serializes a message object of type '<ServoMove>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servo_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'angle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'angle)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServoMove>) istream)
  "Deserializes a message object of type '<ServoMove>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servo_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'angle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'angle)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServoMove>)))
  "Returns string type for a message object of type '<ServoMove>"
  "servo_ros/ServoMove")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoMove)))
  "Returns string type for a message object of type 'ServoMove"
  "servo_ros/ServoMove")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServoMove>)))
  "Returns md5sum for a message object of type '<ServoMove>"
  "fd942452571bdf83e3fa8f79a5e9a7d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServoMove)))
  "Returns md5sum for a message object of type 'ServoMove"
  "fd942452571bdf83e3fa8f79a5e9a7d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServoMove>)))
  "Returns full string definition for message of type '<ServoMove>"
  (cl:format cl:nil "#舵机转动控制~%uint16 servo_id	#舵机ID~%uint16 angle	#角度位置0~~1000~%uint16 time     #运行时间(ms)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServoMove)))
  "Returns full string definition for message of type 'ServoMove"
  (cl:format cl:nil "#舵机转动控制~%uint16 servo_id	#舵机ID~%uint16 angle	#角度位置0~~1000~%uint16 time     #运行时间(ms)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServoMove>))
  (cl:+ 0
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServoMove>))
  "Converts a ROS message object to a list"
  (cl:list 'ServoMove
    (cl:cons ':servo_id (servo_id msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':time (time msg))
))
