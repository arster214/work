;; Auto-generated. Do not edit!


(when (boundp 'dual_arm_msgs::Arm_Software_Version)
  (if (not (find-package "DUAL_ARM_MSGS"))
    (make-package "DUAL_ARM_MSGS"))
  (shadow 'Arm_Software_Version (find-package "DUAL_ARM_MSGS")))
(unless (find-package "DUAL_ARM_MSGS::ARM_SOFTWARE_VERSION")
  (make-package "DUAL_ARM_MSGS::ARM_SOFTWARE_VERSION"))

(in-package "ROS")
;;//! \htmlinclude Arm_Software_Version.msg.html


(defclass dual_arm_msgs::Arm_Software_Version
  :super ros::object
  :slots (_Product_version _Plan_version ))

(defmethod dual_arm_msgs::Arm_Software_Version
  (:init
   (&key
    ((:Product_version __Product_version) "")
    ((:Plan_version __Plan_version) "")
    )
   (send-super :init)
   (setq _Product_version (string __Product_version))
   (setq _Plan_version (string __Plan_version))
   self)
  (:Product_version
   (&optional __Product_version)
   (if __Product_version (setq _Product_version __Product_version)) _Product_version)
  (:Plan_version
   (&optional __Plan_version)
   (if __Plan_version (setq _Plan_version __Plan_version)) _Plan_version)
  (:serialization-length
   ()
   (+
    ;; string _Product_version
    4 (length _Product_version)
    ;; string _Plan_version
    4 (length _Plan_version)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _Product_version
       (write-long (length _Product_version) s) (princ _Product_version s)
     ;; string _Plan_version
       (write-long (length _Plan_version) s) (princ _Plan_version s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _Product_version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Product_version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _Plan_version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _Plan_version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get dual_arm_msgs::Arm_Software_Version :md5sum-) "875ac4a1506e782c8b37a03d7f6b51c9")
(setf (get dual_arm_msgs::Arm_Software_Version :datatype-) "dual_arm_msgs/Arm_Software_Version")
(setf (get dual_arm_msgs::Arm_Software_Version :definition-)
      "string Product_version
string Plan_version

")



(provide :dual_arm_msgs/Arm_Software_Version "875ac4a1506e782c8b37a03d7f6b51c9")


