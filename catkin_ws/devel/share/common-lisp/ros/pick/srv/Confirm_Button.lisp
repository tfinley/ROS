; Auto-generated. Do not edit!


(cl:in-package pick-srv)


;//! \htmlinclude Confirm_Button-request.msg.html

(cl:defclass <Confirm_Button-request> (roslisp-msg-protocol:ros-message)
  ((button
    :reader button
    :initarg :button
    :type cl:string
    :initform ""))
)

(cl:defclass Confirm_Button-request (<Confirm_Button-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Confirm_Button-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Confirm_Button-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pick-srv:<Confirm_Button-request> is deprecated: use pick-srv:Confirm_Button-request instead.")))

(cl:ensure-generic-function 'button-val :lambda-list '(m))
(cl:defmethod button-val ((m <Confirm_Button-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick-srv:button-val is deprecated.  Use pick-srv:button instead.")
  (button m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Confirm_Button-request>) ostream)
  "Serializes a message object of type '<Confirm_Button-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'button))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'button))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Confirm_Button-request>) istream)
  "Deserializes a message object of type '<Confirm_Button-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'button) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'button) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Confirm_Button-request>)))
  "Returns string type for a service object of type '<Confirm_Button-request>"
  "pick/Confirm_ButtonRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Confirm_Button-request)))
  "Returns string type for a service object of type 'Confirm_Button-request"
  "pick/Confirm_ButtonRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Confirm_Button-request>)))
  "Returns md5sum for a message object of type '<Confirm_Button-request>"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Confirm_Button-request)))
  "Returns md5sum for a message object of type 'Confirm_Button-request"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Confirm_Button-request>)))
  "Returns full string definition for message of type '<Confirm_Button-request>"
  (cl:format cl:nil "string button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Confirm_Button-request)))
  "Returns full string definition for message of type 'Confirm_Button-request"
  (cl:format cl:nil "string button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Confirm_Button-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'button))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Confirm_Button-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Confirm_Button-request
    (cl:cons ':button (button msg))
))
;//! \htmlinclude Confirm_Button-response.msg.html

(cl:defclass <Confirm_Button-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass Confirm_Button-response (<Confirm_Button-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Confirm_Button-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Confirm_Button-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pick-srv:<Confirm_Button-response> is deprecated: use pick-srv:Confirm_Button-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <Confirm_Button-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pick-srv:response-val is deprecated.  Use pick-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Confirm_Button-response>) ostream)
  "Serializes a message object of type '<Confirm_Button-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Confirm_Button-response>) istream)
  "Deserializes a message object of type '<Confirm_Button-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Confirm_Button-response>)))
  "Returns string type for a service object of type '<Confirm_Button-response>"
  "pick/Confirm_ButtonResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Confirm_Button-response)))
  "Returns string type for a service object of type 'Confirm_Button-response"
  "pick/Confirm_ButtonResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Confirm_Button-response>)))
  "Returns md5sum for a message object of type '<Confirm_Button-response>"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Confirm_Button-response)))
  "Returns md5sum for a message object of type 'Confirm_Button-response"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Confirm_Button-response>)))
  "Returns full string definition for message of type '<Confirm_Button-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Confirm_Button-response)))
  "Returns full string definition for message of type 'Confirm_Button-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Confirm_Button-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Confirm_Button-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Confirm_Button-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Confirm_Button)))
  'Confirm_Button-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Confirm_Button)))
  'Confirm_Button-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Confirm_Button)))
  "Returns string type for a service object of type '<Confirm_Button>"
  "pick/Confirm_Button")