; Auto-generated. Do not edit!


(cl:in-package lidar_localization-srv)


;//! \htmlinclude saveOdometry-request.msg.html

(cl:defclass <saveOdometry-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass saveOdometry-request (<saveOdometry-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <saveOdometry-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'saveOdometry-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_localization-srv:<saveOdometry-request> is deprecated: use lidar_localization-srv:saveOdometry-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <saveOdometry-request>) ostream)
  "Serializes a message object of type '<saveOdometry-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <saveOdometry-request>) istream)
  "Deserializes a message object of type '<saveOdometry-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<saveOdometry-request>)))
  "Returns string type for a service object of type '<saveOdometry-request>"
  "lidar_localization/saveOdometryRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveOdometry-request)))
  "Returns string type for a service object of type 'saveOdometry-request"
  "lidar_localization/saveOdometryRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<saveOdometry-request>)))
  "Returns md5sum for a message object of type '<saveOdometry-request>"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'saveOdometry-request)))
  "Returns md5sum for a message object of type 'saveOdometry-request"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<saveOdometry-request>)))
  "Returns full string definition for message of type '<saveOdometry-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'saveOdometry-request)))
  "Returns full string definition for message of type 'saveOdometry-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <saveOdometry-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <saveOdometry-request>))
  "Converts a ROS message object to a list"
  (cl:list 'saveOdometry-request
))
;//! \htmlinclude saveOdometry-response.msg.html

(cl:defclass <saveOdometry-response> (roslisp-msg-protocol:ros-message)
  ((succeed
    :reader succeed
    :initarg :succeed
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass saveOdometry-response (<saveOdometry-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <saveOdometry-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'saveOdometry-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_localization-srv:<saveOdometry-response> is deprecated: use lidar_localization-srv:saveOdometry-response instead.")))

(cl:ensure-generic-function 'succeed-val :lambda-list '(m))
(cl:defmethod succeed-val ((m <saveOdometry-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-srv:succeed-val is deprecated.  Use lidar_localization-srv:succeed instead.")
  (succeed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <saveOdometry-response>) ostream)
  "Serializes a message object of type '<saveOdometry-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'succeed) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <saveOdometry-response>) istream)
  "Deserializes a message object of type '<saveOdometry-response>"
    (cl:setf (cl:slot-value msg 'succeed) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<saveOdometry-response>)))
  "Returns string type for a service object of type '<saveOdometry-response>"
  "lidar_localization/saveOdometryResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveOdometry-response)))
  "Returns string type for a service object of type 'saveOdometry-response"
  "lidar_localization/saveOdometryResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<saveOdometry-response>)))
  "Returns md5sum for a message object of type '<saveOdometry-response>"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'saveOdometry-response)))
  "Returns md5sum for a message object of type 'saveOdometry-response"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<saveOdometry-response>)))
  "Returns full string definition for message of type '<saveOdometry-response>"
  (cl:format cl:nil "bool succeed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'saveOdometry-response)))
  "Returns full string definition for message of type 'saveOdometry-response"
  (cl:format cl:nil "bool succeed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <saveOdometry-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <saveOdometry-response>))
  "Converts a ROS message object to a list"
  (cl:list 'saveOdometry-response
    (cl:cons ':succeed (succeed msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'saveOdometry)))
  'saveOdometry-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'saveOdometry)))
  'saveOdometry-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveOdometry)))
  "Returns string type for a service object of type '<saveOdometry>"
  "lidar_localization/saveOdometry")