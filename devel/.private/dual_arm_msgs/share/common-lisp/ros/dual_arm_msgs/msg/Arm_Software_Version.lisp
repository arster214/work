; Auto-generated. Do not edit!


(cl:in-package dual_arm_msgs-msg)


;//! \htmlinclude Arm_Software_Version.msg.html

(cl:defclass <Arm_Software_Version> (roslisp-msg-protocol:ros-message)
  ((Product_version
    :reader Product_version
    :initarg :Product_version
    :type cl:string
    :initform "")
   (Plan_version
    :reader Plan_version
    :initarg :Plan_version
    :type cl:string
    :initform ""))
)

(cl:defclass Arm_Software_Version (<Arm_Software_Version>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Arm_Software_Version>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Arm_Software_Version)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dual_arm_msgs-msg:<Arm_Software_Version> is deprecated: use dual_arm_msgs-msg:Arm_Software_Version instead.")))

(cl:ensure-generic-function 'Product_version-val :lambda-list '(m))
(cl:defmethod Product_version-val ((m <Arm_Software_Version>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dual_arm_msgs-msg:Product_version-val is deprecated.  Use dual_arm_msgs-msg:Product_version instead.")
  (Product_version m))

(cl:ensure-generic-function 'Plan_version-val :lambda-list '(m))
(cl:defmethod Plan_version-val ((m <Arm_Software_Version>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dual_arm_msgs-msg:Plan_version-val is deprecated.  Use dual_arm_msgs-msg:Plan_version instead.")
  (Plan_version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Arm_Software_Version>) ostream)
  "Serializes a message object of type '<Arm_Software_Version>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Product_version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Product_version))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'Plan_version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'Plan_version))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Arm_Software_Version>) istream)
  "Deserializes a message object of type '<Arm_Software_Version>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Product_version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Product_version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Plan_version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'Plan_version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Arm_Software_Version>)))
  "Returns string type for a message object of type '<Arm_Software_Version>"
  "dual_arm_msgs/Arm_Software_Version")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Arm_Software_Version)))
  "Returns string type for a message object of type 'Arm_Software_Version"
  "dual_arm_msgs/Arm_Software_Version")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Arm_Software_Version>)))
  "Returns md5sum for a message object of type '<Arm_Software_Version>"
  "875ac4a1506e782c8b37a03d7f6b51c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Arm_Software_Version)))
  "Returns md5sum for a message object of type 'Arm_Software_Version"
  "875ac4a1506e782c8b37a03d7f6b51c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Arm_Software_Version>)))
  "Returns full string definition for message of type '<Arm_Software_Version>"
  (cl:format cl:nil "string Product_version~%string Plan_version~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Arm_Software_Version)))
  "Returns full string definition for message of type 'Arm_Software_Version"
  (cl:format cl:nil "string Product_version~%string Plan_version~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Arm_Software_Version>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'Product_version))
     4 (cl:length (cl:slot-value msg 'Plan_version))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Arm_Software_Version>))
  "Converts a ROS message object to a list"
  (cl:list 'Arm_Software_Version
    (cl:cons ':Product_version (Product_version msg))
    (cl:cons ':Plan_version (Plan_version msg))
))
