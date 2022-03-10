; Auto-generated. Do not edit!


(cl:in-package lidar_localization-srv)


;//! \htmlinclude saveMap-request.msg.html

(cl:defclass <saveMap-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass saveMap-request (<saveMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <saveMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'saveMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_localization-srv:<saveMap-request> is deprecated: use lidar_localization-srv:saveMap-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <saveMap-request>) ostream)
  "Serializes a message object of type '<saveMap-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <saveMap-request>) istream)
  "Deserializes a message object of type '<saveMap-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<saveMap-request>)))
  "Returns string type for a service object of type '<saveMap-request>"
  "lidar_localization/saveMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveMap-request)))
  "Returns string type for a service object of type 'saveMap-request"
  "lidar_localization/saveMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<saveMap-request>)))
  "Returns md5sum for a message object of type '<saveMap-request>"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'saveMap-request)))
  "Returns md5sum for a message object of type 'saveMap-request"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<saveMap-request>)))
  "Returns full string definition for message of type '<saveMap-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'saveMap-request)))
  "Returns full string definition for message of type 'saveMap-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <saveMap-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <saveMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'saveMap-request
))
;//! \htmlinclude saveMap-response.msg.html

(cl:defclass <saveMap-response> (roslisp-msg-protocol:ros-message)
  ((succeed
    :reader succeed
    :initarg :succeed
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass saveMap-response (<saveMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <saveMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'saveMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_localization-srv:<saveMap-response> is deprecated: use lidar_localization-srv:saveMap-response instead.")))

(cl:ensure-generic-function 'succeed-val :lambda-list '(m))
(cl:defmethod succeed-val ((m <saveMap-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-srv:succeed-val is deprecated.  Use lidar_localization-srv:succeed instead.")
  (succeed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <saveMap-response>) ostream)
  "Serializes a message object of type '<saveMap-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'succeed) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <saveMap-response>) istream)
  "Deserializes a message object of type '<saveMap-response>"
    (cl:setf (cl:slot-value msg 'succeed) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<saveMap-response>)))
  "Returns string type for a service object of type '<saveMap-response>"
  "lidar_localization/saveMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveMap-response)))
  "Returns string type for a service object of type 'saveMap-response"
  "lidar_localization/saveMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<saveMap-response>)))
  "Returns md5sum for a message object of type '<saveMap-response>"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'saveMap-response)))
  "Returns md5sum for a message object of type 'saveMap-response"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<saveMap-response>)))
  "Returns full string definition for message of type '<saveMap-response>"
  (cl:format cl:nil "bool succeed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'saveMap-response)))
  "Returns full string definition for message of type 'saveMap-response"
  (cl:format cl:nil "bool succeed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <saveMap-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <saveMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'saveMap-response
    (cl:cons ':succeed (succeed msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'saveMap)))
  'saveMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'saveMap)))
  'saveMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveMap)))
  "Returns string type for a service object of type '<saveMap>"
  "lidar_localization/saveMap")