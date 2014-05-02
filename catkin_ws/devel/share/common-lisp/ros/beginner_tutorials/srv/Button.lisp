; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-srv)


;//! \htmlinclude Button-request.msg.html

(cl:defclass <Button-request> (roslisp-msg-protocol:ros-message)
  ((button
    :reader button
    :initarg :button
    :type cl:string
    :initform ""))
)

(cl:defclass Button-request (<Button-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Button-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Button-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-srv:<Button-request> is deprecated: use beginner_tutorials-srv:Button-request instead.")))

(cl:ensure-generic-function 'button-val :lambda-list '(m))
(cl:defmethod button-val ((m <Button-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-srv:button-val is deprecated.  Use beginner_tutorials-srv:button instead.")
  (button m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Button-request>) ostream)
  "Serializes a message object of type '<Button-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'button))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'button))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Button-request>) istream)
  "Deserializes a message object of type '<Button-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Button-request>)))
  "Returns string type for a service object of type '<Button-request>"
  "beginner_tutorials/ButtonRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Button-request)))
  "Returns string type for a service object of type 'Button-request"
  "beginner_tutorials/ButtonRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Button-request>)))
  "Returns md5sum for a message object of type '<Button-request>"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Button-request)))
  "Returns md5sum for a message object of type 'Button-request"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Button-request>)))
  "Returns full string definition for message of type '<Button-request>"
  (cl:format cl:nil "string button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Button-request)))
  "Returns full string definition for message of type 'Button-request"
  (cl:format cl:nil "string button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Button-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'button))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Button-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Button-request
    (cl:cons ':button (button msg))
))
;//! \htmlinclude Button-response.msg.html

(cl:defclass <Button-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass Button-response (<Button-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Button-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Button-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-srv:<Button-response> is deprecated: use beginner_tutorials-srv:Button-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <Button-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-srv:response-val is deprecated.  Use beginner_tutorials-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Button-response>) ostream)
  "Serializes a message object of type '<Button-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Button-response>) istream)
  "Deserializes a message object of type '<Button-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Button-response>)))
  "Returns string type for a service object of type '<Button-response>"
  "beginner_tutorials/ButtonResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Button-response)))
  "Returns string type for a service object of type 'Button-response"
  "beginner_tutorials/ButtonResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Button-response>)))
  "Returns md5sum for a message object of type '<Button-response>"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Button-response)))
  "Returns md5sum for a message object of type 'Button-response"
  "fce55d8d6e1bc1923599409813fddc68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Button-response>)))
  "Returns full string definition for message of type '<Button-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Button-response)))
  "Returns full string definition for message of type 'Button-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Button-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Button-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Button-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Button)))
  'Button-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Button)))
  'Button-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Button)))
  "Returns string type for a service object of type '<Button>"
  "beginner_tutorials/Button")