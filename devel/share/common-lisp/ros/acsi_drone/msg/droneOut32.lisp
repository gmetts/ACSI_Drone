; Auto-generated. Do not edit!


(cl:in-package acsi_drone-msg)


;//! \htmlinclude droneOut32.msg.html

(cl:defclass <droneOut32> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass droneOut32 (<droneOut32>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <droneOut32>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'droneOut32)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name acsi_drone-msg:<droneOut32> is deprecated: use acsi_drone-msg:droneOut32 instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <droneOut32>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader acsi_drone-msg:data-val is deprecated.  Use acsi_drone-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <droneOut32>) ostream)
  "Serializes a message object of type '<droneOut32>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <droneOut32>) istream)
  "Deserializes a message object of type '<droneOut32>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<droneOut32>)))
  "Returns string type for a message object of type '<droneOut32>"
  "acsi_drone/droneOut32")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'droneOut32)))
  "Returns string type for a message object of type 'droneOut32"
  "acsi_drone/droneOut32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<droneOut32>)))
  "Returns md5sum for a message object of type '<droneOut32>"
  "420cd38b6b071cd49f2970c3e2cee511")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'droneOut32)))
  "Returns md5sum for a message object of type 'droneOut32"
  "420cd38b6b071cd49f2970c3e2cee511")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<droneOut32>)))
  "Returns full string definition for message of type '<droneOut32>"
  (cl:format cl:nil "float32[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'droneOut32)))
  "Returns full string definition for message of type 'droneOut32"
  (cl:format cl:nil "float32[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <droneOut32>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <droneOut32>))
  "Converts a ROS message object to a list"
  (cl:list 'droneOut32
    (cl:cons ':data (data msg))
))
