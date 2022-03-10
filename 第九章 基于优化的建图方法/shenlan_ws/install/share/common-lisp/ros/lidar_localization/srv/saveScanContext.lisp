; Auto-generated. Do not edit!


(cl:in-package lidar_localization-srv)


;//! \htmlinclude saveScanContext-request.msg.html

(cl:defclass <saveScanContext-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass saveScanContext-request (<saveScanContext-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <saveScanContext-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'saveScanContext-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_localization-srv:<saveScanContext-request> is deprecated: use lidar_localization-srv:saveScanContext-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <saveScanContext-request>) ostream)
  "Serializes a message object of type '<saveScanContext-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <saveScanContext-request>) istream)
  "Deserializes a message object of type '<saveScanContext-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<saveScanContext-request>)))
  "Returns string type for a service object of type '<saveScanContext-request>"
  "lidar_localization/saveScanContextRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveScanContext-request)))
  "Returns string type for a service object of type 'saveScanContext-request"
  "lidar_localization/saveScanContextRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<saveScanContext-request>)))
  "Returns md5sum for a message object of type '<saveScanContext-request>"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'saveScanContext-request)))
  "Returns md5sum for a message object of type 'saveScanContext-request"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<saveScanContext-request>)))
  "Returns full string definition for message of type '<saveScanContext-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'saveScanContext-request)))
  "Returns full string definition for message of type 'saveScanContext-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <saveScanContext-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <saveScanContext-request>))
  "Converts a ROS message object to a list"
  (cl:list 'saveScanContext-request
))
;//! \htmlinclude saveScanContext-response.msg.html

(cl:defclass <saveScanContext-response> (roslisp-msg-protocol:ros-message)
  ((succeed
    :reader succeed
    :initarg :succeed
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass saveScanContext-response (<saveScanContext-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <saveScanContext-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'saveScanContext-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_localization-srv:<saveScanContext-response> is deprecated: use lidar_localization-srv:saveScanContext-response instead.")))

(cl:ensure-generic-function 'succeed-val :lambda-list '(m))
(cl:defmethod succeed-val ((m <saveScanContext-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_localization-srv:succeed-val is deprecated.  Use lidar_localization-srv:succeed instead.")
  (succeed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <saveScanContext-response>) ostream)
  "Serializes a message object of type '<saveScanContext-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'succeed) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <saveScanContext-response>) istream)
  "Deserializes a message object of type '<saveScanContext-response>"
    (cl:setf (cl:slot-value msg 'succeed) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<saveScanContext-response>)))
  "Returns string type for a service object of type '<saveScanContext-response>"
  "lidar_localization/saveScanContextResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveScanContext-response)))
  "Returns string type for a service object of type 'saveScanContext-response"
  "lidar_localization/saveScanContextResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<saveScanContext-response>)))
  "Returns md5sum for a message object of type '<saveScanContext-response>"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'saveScanContext-response)))
  "Returns md5sum for a message object of type 'saveScanContext-response"
  "8d9c3b918a0afafe09791ef8d7853918")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<saveScanContext-response>)))
  "Returns full string definition for message of type '<saveScanContext-response>"
  (cl:format cl:nil "bool succeed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'saveScanContext-response)))
  "Returns full string definition for message of type 'saveScanContext-response"
  (cl:format cl:nil "bool succeed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <saveScanContext-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <saveScanContext-response>))
  "Converts a ROS message object to a list"
  (cl:list 'saveScanContext-response
    (cl:cons ':succeed (succeed msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'saveScanContext)))
  'saveScanContext-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'saveScanContext)))
  'saveScanContext-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'saveScanContext)))
  "Returns string type for a service object of type '<saveScanContext>"
  "lidar_localization/saveScanContext")