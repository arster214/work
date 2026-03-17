; Auto-generated. Do not edit!


(cl:in-package servo_ros-msg)


;//! \htmlinclude ServoAngle.msg.html

(cl:defclass <ServoAngle> (roslisp-msg-protocol:ros-message)
  ((servo_id_1
    :reader servo_id_1
    :initarg :servo_id_1
    :type cl:fixnum
    :initform 0)
   (angle_1
    :reader angle_1
    :initarg :angle_1
    :type cl:fixnum
    :initform 0)
   (servo_id_2
    :reader servo_id_2
    :initarg :servo_id_2
    :type cl:fixnum
    :initform 0)
   (angle_2
    :reader angle_2
    :initarg :angle_2
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ServoAngle (<ServoAngle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ServoAngle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ServoAngle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name servo_ros-msg:<ServoAngle> is deprecated: use servo_ros-msg:ServoAngle instead.")))

(cl:ensure-generic-function 'servo_id_1-val :lambda-list '(m))
(cl:defmethod servo_id_1-val ((m <ServoAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_ros-msg:servo_id_1-val is deprecated.  Use servo_ros-msg:servo_id_1 instead.")
  (servo_id_1 m))

(cl:ensure-generic-function 'angle_1-val :lambda-list '(m))
(cl:defmethod angle_1-val ((m <ServoAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_ros-msg:angle_1-val is deprecated.  Use servo_ros-msg:angle_1 instead.")
  (angle_1 m))

(cl:ensure-generic-function 'servo_id_2-val :lambda-list '(m))
(cl:defmethod servo_id_2-val ((m <ServoAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_ros-msg:servo_id_2-val is deprecated.  Use servo_ros-msg:servo_id_2 instead.")
  (servo_id_2 m))

(cl:ensure-generic-function 'angle_2-val :lambda-list '(m))
(cl:defmethod angle_2-val ((m <ServoAngle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader servo_ros-msg:angle_2-val is deprecated.  Use servo_ros-msg:angle_2 instead.")
  (angle_2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ServoAngle>) ostream)
  "Serializes a message object of type '<ServoAngle>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_id_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servo_id_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'angle_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'angle_1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_id_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servo_id_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'angle_2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'angle_2)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ServoAngle>) istream)
  "Deserializes a message object of type '<ServoAngle>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_id_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servo_id_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'angle_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'angle_1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servo_id_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servo_id_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'angle_2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'angle_2)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ServoAngle>)))
  "Returns string type for a message object of type '<ServoAngle>"
  "servo_ros/ServoAngle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ServoAngle)))
  "Returns string type for a message object of type 'ServoAngle"
  "servo_ros/ServoAngle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ServoAngle>)))
  "Returns md5sum for a message object of type '<ServoAngle>"
  "9a8103b4cee119f9447f2067818f6250")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ServoAngle)))
  "Returns md5sum for a message object of type 'ServoAngle"
  "9a8103b4cee119f9447f2067818f6250")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ServoAngle>)))
  "Returns full string definition for message of type '<ServoAngle>"
  (cl:format cl:nil "#舵机返回角度~%uint16 servo_id_1	#舵机ID~%uint16 angle_1	#角度位置0~~1000~%uint16 servo_id_2	#舵机ID~%uint16 angle_2	#角度位置0~~1000~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ServoAngle)))
  "Returns full string definition for message of type 'ServoAngle"
  (cl:format cl:nil "#舵机返回角度~%uint16 servo_id_1	#舵机ID~%uint16 angle_1	#角度位置0~~1000~%uint16 servo_id_2	#舵机ID~%uint16 angle_2	#角度位置0~~1000~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ServoAngle>))
  (cl:+ 0
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ServoAngle>))
  "Converts a ROS message object to a list"
  (cl:list 'ServoAngle
    (cl:cons ':servo_id_1 (servo_id_1 msg))
    (cl:cons ':angle_1 (angle_1 msg))
    (cl:cons ':servo_id_2 (servo_id_2 msg))
    (cl:cons ':angle_2 (angle_2 msg))
))
