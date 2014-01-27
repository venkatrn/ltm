; Auto-generated. Do not edit!


(cl:in-package ltm_msgs-msg)


;//! \htmlinclude DModel.msg.html

(cl:defclass <DModel> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray))
   (edges
    :reader edges
    :initarg :edges
    :type (cl:vector ltm_msgs-msg:Edge)
   :initform (cl:make-array 0 :element-type 'ltm_msgs-msg:Edge :initial-element (cl:make-instance 'ltm_msgs-msg:Edge))))
)

(cl:defclass DModel (<DModel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DModel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DModel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ltm_msgs-msg:<DModel> is deprecated: use ltm_msgs-msg:DModel instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <DModel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltm_msgs-msg:points-val is deprecated.  Use ltm_msgs-msg:points instead.")
  (points m))

(cl:ensure-generic-function 'edges-val :lambda-list '(m))
(cl:defmethod edges-val ((m <DModel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ltm_msgs-msg:edges-val is deprecated.  Use ltm_msgs-msg:edges instead.")
  (edges m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DModel>) ostream)
  "Serializes a message object of type '<DModel>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'points) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'edges))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'edges))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DModel>) istream)
  "Deserializes a message object of type '<DModel>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'points) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'edges) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'edges)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ltm_msgs-msg:Edge))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DModel>)))
  "Returns string type for a message object of type '<DModel>"
  "ltm_msgs/DModel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DModel)))
  "Returns string type for a message object of type 'DModel"
  "ltm_msgs/DModel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DModel>)))
  "Returns md5sum for a message object of type '<DModel>"
  "50a566339aa89b9aa728558673bc6645")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DModel)))
  "Returns md5sum for a message object of type 'DModel"
  "50a566339aa89b9aa728558673bc6645")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DModel>)))
  "Returns full string definition for message of type '<DModel>"
  (cl:format cl:nil "geometry_msgs/PoseArray points~%ltm_msgs/Edge[] edges~%~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: ltm_msgs/Edge~%int64[2] nodes~%int64 joint_type~%float64[3] normal~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DModel)))
  "Returns full string definition for message of type 'DModel"
  (cl:format cl:nil "geometry_msgs/PoseArray points~%ltm_msgs/Edge[] edges~%~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: ltm_msgs/Edge~%int64[2] nodes~%int64 joint_type~%float64[3] normal~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DModel>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'points))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'edges) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DModel>))
  "Converts a ROS message object to a list"
  (cl:list 'DModel
    (cl:cons ':points (points msg))
    (cl:cons ':edges (edges msg))
))
