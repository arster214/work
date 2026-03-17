; Auto-generated. Do not edit!


(cl:in-package dual_arm_msgs-msg)


;//! \htmlinclude CarteFdPose.msg.html

(cl:defclass <CarteFdPose> (roslisp-msg-protocol:ros-message)
  ((Pose_Euler
    :reader Pose_Euler
    :initarg :Pose_Euler
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass CarteFdPose (<CarteFdPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CarteFdPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CarteFdPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dual_arm_msgs-msg:<CarteFdPose> is deprecated: use dual_arm_msgs-msg:CarteFdPose instead.")))

(cl:ensure-generic-function 'Pose_Euler-val :lambda-list '(m))
(cl:defmethod Pose_Euler-val ((m <CarteFdPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dual_arm_msgs-msg:Pose_Euler-val is deprecated.  Use dual_arm_msgs-msg:Pose_Euler instead.")
  (Pose_Euler m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CarteFdPose>) ostream)
  "Serializes a message object of type '<CarteFdPose>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Pose_Euler))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CarteFdPose>) istream)
  "Deserializes a message object of type '<CarteFdPose>"
  (cl:setf (cl:slot-value msg 'Pose_Euler) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'Pose_Euler)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CarteFdPose>)))
  "Returns string type for a message object of type '<CarteFdPose>"
  "dual_arm_msgs/CarteFdPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CarteFdPose)))
  "Returns string type for a message object of type 'CarteFdPose"
  "dual_arm_msgs/CarteFdPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CarteFdPose>)))
  "Returns md5sum for a message object of type '<CarteFdPose>"
  "525ed60ead11361f867b19c646e09073")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CarteFdPose)))
  "Returns md5sum for a message object of type 'CarteFdPose"
  "525ed60ead11361f867b19c646e09073")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CarteFdPose>)))
  "Returns full string definition for message of type '<CarteFdPose>"
  (cl:format cl:nil "float32[6] Pose_Euler~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CarteFdPose)))
  "Returns full string definition for message of type 'CarteFdPose"
  (cl:format cl:nil "float32[6] Pose_Euler~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CarteFdPose>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'Pose_Euler) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CarteFdPose>))
  "Converts a ROS message object to a list"
  (cl:list 'CarteFdPose
    (cl:cons ':Pose_Euler (Pose_Euler msg))
))
