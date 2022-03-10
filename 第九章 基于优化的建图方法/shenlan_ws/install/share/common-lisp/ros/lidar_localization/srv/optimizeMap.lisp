; Auto-generated. Do not edit!


(cl:in-package lidar_localization-srv)


;//! \htmlinclude optimizeMap-request.msg.html

(cl:defclass <optimizeMap-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass optimizeMap-request (<optimizeMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <optimizeMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'optimizeMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_localization-srv:<optimizeMap-request> is deprecated: use lidar_localization-srv:optimizeMap-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <optimizeMap-request>) ostream)
  "Serializes a message object of type '<optimizeMap-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <optimizeMap-request>) istream)
  "Deserializes a message object of type '<optimizeMap-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<optimizeMap-request>)))
  "Returns string type for a service object of type '<optimizeMap-request>"
  "lidar_localization/optimizeMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'optimizeMap-request)))
  "Returns string type for a service object of type 'optimizeMap-request"
  "lidar_localization/optimizeMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<optimizeMap-request>)))
  "Returns md5sum for a message object of type '<optimizeMap-request>"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'optimizeMap-request)))
  "Returns md5sum for a message object of type 'optimizeMap-request"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<optimizeMap-request>)))
  "Returns full string definition for message of type '<optimizeMap-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'optimizeMap-request)))
  "Returns full string definition for message of type 'optimizeMap-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <optimizeMap-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <optimizeMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'optimizeMap-request
))
;//! \htmlinclude optimizeMap-response.msg.html

(cl:defclass <optimizeMap-response> (roslisp-msg-protocol:ros-message)
  ((succeed
    :reader succeed
    :initarg :succeed
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass optimizeMap-response (<optimizeMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <optimizeMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'optimizeMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_localization-srv:<optimizeMap-response> is deprecated: use lidar_localization-srv:optimizeMap-response instead.")))

(cl:ensure-generic-function 'succeed-val :lambda-list '(m))
(cl:defmethod succeed-val ((m <optimizeMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-srv:succeed-val is deprecated.  Use lidar_localization-srv:succeed instead.")
  (succeed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <optimizeMap-response>) ostream)
  "Serializes a message object of type '<optimizeMap-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'succeed) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <optimizeMap-response>) istream)
  "Deserializes a message object of type '<optimizeMap-response>"
    (cl:setf (cl:slot-value msg 'succeed) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<optimizeMap-response>)))
  "Returns string type for a service object of type '<optimizeMap-response>"
  "lidar_localization/optimizeMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'optimizeMap-response)))
  "Returns string type for a service object of type 'optimizeMap-response"
  "lidar_localization/optimizeMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<optimizeMap-response>)))
  "Returns md5sum for a message object of type '<optimizeMap-response>"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'optimizeMap-response)))
  "Returns md5sum for a message object of type 'optimizeMap-response"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<optimizeMap-response>)))
  "Returns full string definition for message of type '<optimizeMap-response>"
  (cl:format cl:nil "bool succeed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'optimizeMap-response)))
  "Returns full string definition for message of type 'optimizeMap-response"
  (cl:format cl:nil "bool succeed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <optimizeMap-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <optimizeMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'optimizeMap-response
    (cl:cons ':succeed (succeed msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'optimizeMap)))
  'optimizeMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'optimizeMap)))
  'optimizeMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'optimizeMap)))
  "Returns string type for a service object of type '<optimizeMap>"
  "lidar_localization/optimizeMap")