// Auto-generated. Do not edit!

// (in-package controller_modules.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let trajectory_msgs = _finder('trajectory_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class JointControlRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.joint_names = null;
      this.controller_name = null;
      this.desired = null;
      this.actual = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('joint_names')) {
        this.joint_names = initObj.joint_names
      }
      else {
        this.joint_names = [];
      }
      if (initObj.hasOwnProperty('controller_name')) {
        this.controller_name = initObj.controller_name
      }
      else {
        this.controller_name = '';
      }
      if (initObj.hasOwnProperty('desired')) {
        this.desired = initObj.desired
      }
      else {
        this.desired = new trajectory_msgs.msg.JointTrajectoryPoint();
      }
      if (initObj.hasOwnProperty('actual')) {
        this.actual = initObj.actual
      }
      else {
        this.actual = new trajectory_msgs.msg.JointTrajectoryPoint();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointControlRequest
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [joint_names]
    bufferOffset = _arraySerializer.string(obj.joint_names, buffer, bufferOffset, null);
    // Serialize message field [controller_name]
    bufferOffset = _serializer.string(obj.controller_name, buffer, bufferOffset);
    // Serialize message field [desired]
    bufferOffset = trajectory_msgs.msg.JointTrajectoryPoint.serialize(obj.desired, buffer, bufferOffset);
    // Serialize message field [actual]
    bufferOffset = trajectory_msgs.msg.JointTrajectoryPoint.serialize(obj.actual, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointControlRequest
    let len;
    let data = new JointControlRequest(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_names]
    data.joint_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [controller_name]
    data.controller_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [desired]
    data.desired = trajectory_msgs.msg.JointTrajectoryPoint.deserialize(buffer, bufferOffset);
    // Deserialize message field [actual]
    data.actual = trajectory_msgs.msg.JointTrajectoryPoint.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.joint_names.forEach((val) => {
      length += 4 + val.length;
    });
    length += object.controller_name.length;
    length += trajectory_msgs.msg.JointTrajectoryPoint.getMessageSize(object.desired);
    length += trajectory_msgs.msg.JointTrajectoryPoint.getMessageSize(object.actual);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'controller_modules/JointControlRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec0f8ccf443b2f5f131ee015d59e16d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string[] joint_names
    string controller_name
    trajectory_msgs/JointTrajectoryPoint desired
    trajectory_msgs/JointTrajectoryPoint actual
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: trajectory_msgs/JointTrajectoryPoint
    # Each trajectory point specifies either positions[, velocities[, accelerations]]
    # or positions[, effort] for the trajectory to be executed.
    # All specified values are in the same order as the joint names in JointTrajectory.msg
    
    float64[] positions
    float64[] velocities
    float64[] accelerations
    float64[] effort
    duration time_from_start
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointControlRequest(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.joint_names !== undefined) {
      resolved.joint_names = msg.joint_names;
    }
    else {
      resolved.joint_names = []
    }

    if (msg.controller_name !== undefined) {
      resolved.controller_name = msg.controller_name;
    }
    else {
      resolved.controller_name = ''
    }

    if (msg.desired !== undefined) {
      resolved.desired = trajectory_msgs.msg.JointTrajectoryPoint.Resolve(msg.desired)
    }
    else {
      resolved.desired = new trajectory_msgs.msg.JointTrajectoryPoint()
    }

    if (msg.actual !== undefined) {
      resolved.actual = trajectory_msgs.msg.JointTrajectoryPoint.Resolve(msg.actual)
    }
    else {
      resolved.actual = new trajectory_msgs.msg.JointTrajectoryPoint()
    }

    return resolved;
    }
};

class JointControlResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.control_output = null;
    }
    else {
      if (initObj.hasOwnProperty('control_output')) {
        this.control_output = initObj.control_output
      }
      else {
        this.control_output = new trajectory_msgs.msg.JointTrajectoryPoint();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointControlResponse
    // Serialize message field [control_output]
    bufferOffset = trajectory_msgs.msg.JointTrajectoryPoint.serialize(obj.control_output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointControlResponse
    let len;
    let data = new JointControlResponse(null);
    // Deserialize message field [control_output]
    data.control_output = trajectory_msgs.msg.JointTrajectoryPoint.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += trajectory_msgs.msg.JointTrajectoryPoint.getMessageSize(object.control_output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'controller_modules/JointControlResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ec3ebfca49414b25763484f58b5180b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    trajectory_msgs/JointTrajectoryPoint control_output
    
    ================================================================================
    MSG: trajectory_msgs/JointTrajectoryPoint
    # Each trajectory point specifies either positions[, velocities[, accelerations]]
    # or positions[, effort] for the trajectory to be executed.
    # All specified values are in the same order as the joint names in JointTrajectory.msg
    
    float64[] positions
    float64[] velocities
    float64[] accelerations
    float64[] effort
    duration time_from_start
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointControlResponse(null);
    if (msg.control_output !== undefined) {
      resolved.control_output = trajectory_msgs.msg.JointTrajectoryPoint.Resolve(msg.control_output)
    }
    else {
      resolved.control_output = new trajectory_msgs.msg.JointTrajectoryPoint()
    }

    return resolved;
    }
};

module.exports = {
  Request: JointControlRequest,
  Response: JointControlResponse,
  md5sum() { return '48f8bb00f33b7039bd35678b2c6204e5'; },
  datatype() { return 'controller_modules/JointControl'; }
};
