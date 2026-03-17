; Auto-generated. Do not edit!


(cl:in-package dual_arm_msgs-msg)


;//! \htmlinclude Arm_Pose_Euler.msg.html

(cl:defclass <Arm_Pose_Euler> (roslisp-msg-protocol:ros-message)
  ((Px
    :reader Px
    :initarg :Px
    :type cl:float
    :initform 0.0)
   (Py
    :reader Py
    :initarg :Py
    :type cl:float
    :initform 0.0)
   (Pz
    :reader Pz
    :initarg :Pz
    :type cl:float
    :initform 0.0)
   (Rx
    :reader Rx
    :initarg :Rx
    :type cl:float
    :initform 0.0)
   (Ry
    :reader Ry
    :initarg :Ry
    :type cl:float
    :initform 0.0)
   (Rz
    :reader Rz
    :initarg :Rz
    :type cl:float
    :initform 0.0))
)

(cl:defclass Arm_Pose_Euler (<Arm_Pose_Euler>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Arm_Pose_Euler>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Arm_Pose_Euler)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dual_arm_msgs-msg:<Arm_Pose_Euler> is deprecated: use dual_arm_msgs-msg:Arm_Pose_Euler instead.")))

(cl:ensure-generic-function 'Px-val :lambda-list '(m))
(cl:defmethod Px-val ((m <Arm_Pose_Euler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dual_arm_msgs-msg:Px-val is deprecated.  Use dual_arm_msgs-msg:Px instead.")
  (Px m))

(cl:ensure-generic-function 'Py-val :lambda-list '(m))
(cl:defmethod Py-val ((m <Arm_Pose_Euler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dual_arm_msgs-msg:Py-val is deprecated.  Use dual_arm_msgs-msg:Py instead.")
  (Py m))

(cl:ensure-generic-function 'Pz-val :lambda-list '(m))
(cl:defmethod Pz-val ((m <Arm_Pose_Euler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dual_arm_msgs-msg:Pz-val is deprecated.  Use dual_arm_msgs-msg:Pz instead.")
  (Pz m))

(cl:ensure-generic-function 'Rx-val :lambda-list '(m))
(cl:defmethod Rx-val ((m <Arm_Pose_Euler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dual_arm_msgs-msg:Rx-val is deprecated.  Use dual_arm_msgs-msg:Rx instead.")
  (Rx m))

(cl:ensure-generic-function 'Ry-val :lambda-list '(m))
(cl:defmethod Ry-val ((m <Arm_Pose_Euler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dual_arm_msgs-msg:Ry-val is deprecated.  Use dual_arm_msgs-msg:Ry instead.")
  (Ry m))

(cl:ensure-generic-function 'Rz-val :lambda-list '(m))
(cl:defmethod Rz-val ((m <Arm_Pose_Euler>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dual_arm_msgs-msg:Rz-val is deprecated.  Use dual_arm_msgs-msg:Rz instead.")
  (Rz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Arm_Pose_Euler>) ostream)
  "Serializes a message object of type '<Arm_Pose_Euler>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Py))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Pz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Rx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Ry))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Rz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Arm_Pose_Euler>) istream)
  "Deserializes a message object of type '<Arm_Pose_Euler>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Px) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Py) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Pz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Rx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Ry) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Rz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Arm_Pose_Euler>)))
  "Returns string type for a message object of type '<Arm_Pose_Euler>"
  "dual_arm_msgs/Arm_Pose_Euler")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Arm_Pose_Euler)))
  "Returns string type for a message object of type 'Arm_Pose_Euler"
  "dual_arm_msgs/Arm_Pose_Euler")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Arm_Pose_Euler>)))
  "Returns md5sum for a message object of type '<Arm_Pose_Euler>"
  "14cf69e90ba61a0c7ae2933996244f58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Arm_Pose_Euler)))
  "Returns md5sum for a message object of type 'Arm_Pose_Euler"
  "14cf69e90ba61a0c7ae2933996244f58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Arm_Pose_Euler>)))
  "Returns full string definition for message of type '<Arm_Pose_Euler>"
  (cl:format cl:nil "float32 Px~%float32 Py~%float32 Pz~%float32 Rx~%float32 Ry~%float32 Rz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Arm_Pose_Euler)))
  "Returns full string definition for message of type 'Arm_Pose_Euler"
  (cl:format cl:nil "float32 Px~%float32 Py~%float32 Pz~%float32 Rx~%float32 Ry~%float32 Rz~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Arm_Pose_Euler>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Arm_Pose_Euler>))
  "Converts a ROS message object to a list"
  (cl:list 'Arm_Pose_Euler
    (cl:cons ':Px (Px msg))
    (cl:cons ':Py (Py msg))
    (cl:cons ':Pz (Pz msg))
    (cl:cons ':Rx (Rx msg))
    (cl:cons ':Ry (Ry msg))
    (cl:cons ':Rz (Rz msg))
))
