; Auto-generated. Do not edit!


(cl:in-package morai_msgs-msg)


;//! \htmlinclude MultiPlayEventResponse.msg.html

(cl:defclass <MultiPlayEventResponse> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass MultiPlayEventResponse (<MultiPlayEventResponse>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MultiPlayEventResponse>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MultiPlayEventResponse)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name morai_msgs-msg:<MultiPlayEventResponse> is deprecated: use morai_msgs-msg:MultiPlayEventResponse instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <MultiPlayEventResponse>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader morai_msgs-msg:result-val is deprecated.  Use morai_msgs-msg:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MultiPlayEventResponse>) ostream)
  "Serializes a message object of type '<MultiPlayEventResponse>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MultiPlayEventResponse>) istream)
  "Deserializes a message object of type '<MultiPlayEventResponse>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MultiPlayEventResponse>)))
  "Returns string type for a message object of type '<MultiPlayEventResponse>"
  "morai_msgs/MultiPlayEventResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MultiPlayEventResponse)))
  "Returns string type for a message object of type 'MultiPlayEventResponse"
  "morai_msgs/MultiPlayEventResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MultiPlayEventResponse>)))
  "Returns md5sum for a message object of type '<MultiPlayEventResponse>"
  "eb13ac1f1354ccecb7941ee8fa2192e8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MultiPlayEventResponse)))
  "Returns md5sum for a message object of type 'MultiPlayEventResponse"
  "eb13ac1f1354ccecb7941ee8fa2192e8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MultiPlayEventResponse>)))
  "Returns full string definition for message of type '<MultiPlayEventResponse>"
  (cl:format cl:nil "bool result~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MultiPlayEventResponse)))
  "Returns full string definition for message of type 'MultiPlayEventResponse"
  (cl:format cl:nil "bool result~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MultiPlayEventResponse>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MultiPlayEventResponse>))
  "Converts a ROS message object to a list"
  (cl:list 'MultiPlayEventResponse
    (cl:cons ':result (result msg))
))
