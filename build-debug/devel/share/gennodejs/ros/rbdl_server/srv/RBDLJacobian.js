// Auto-generated. Do not edit!

// (in-package rbdl_server.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RBDLJacobianRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.body_name = null;
      this.q = null;
      this.point = null;
    }
    else {
      if (initObj.hasOwnProperty('body_name')) {
        this.body_name = initObj.body_name
      }
      else {
        this.body_name = '';
      }
      if (initObj.hasOwnProperty('q')) {
        this.q = initObj.q
      }
      else {
        this.q = [];
      }
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RBDLJacobianRequest
    // Serialize message field [body_name]
    bufferOffset = _serializer.string(obj.body_name, buffer, bufferOffset);
    // Serialize message field [q]
    bufferOffset = _arraySerializer.float64(obj.q, buffer, bufferOffset, null);
    // Serialize message field [point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RBDLJacobianRequest
    let len;
    let data = new RBDLJacobianRequest(null);
    // Deserialize message field [body_name]
    data.body_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [q]
    data.q = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [point]
    data.point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.body_name.length;
    length += 8 * object.q.length;
    return length + 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rbdl_server/RBDLJacobianRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c044abd2a124ba484feaebd59aff30cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string body_name
    float64[] q
    geometry_msgs/Point point
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RBDLJacobianRequest(null);
    if (msg.body_name !== undefined) {
      resolved.body_name = msg.body_name;
    }
    else {
      resolved.body_name = ''
    }

    if (msg.q !== undefined) {
      resolved.q = msg.q;
    }
    else {
      resolved.q = []
    }

    if (msg.point !== undefined) {
      resolved.point = geometry_msgs.msg.Point.Resolve(msg.point)
    }
    else {
      resolved.point = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

class RBDLJacobianResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.jacobian = null;
    }
    else {
      if (initObj.hasOwnProperty('jacobian')) {
        this.jacobian = initObj.jacobian
      }
      else {
        this.jacobian = new std_msgs.msg.Float64MultiArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RBDLJacobianResponse
    // Serialize message field [jacobian]
    bufferOffset = std_msgs.msg.Float64MultiArray.serialize(obj.jacobian, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RBDLJacobianResponse
    let len;
    let data = new RBDLJacobianResponse(null);
    // Deserialize message field [jacobian]
    data.jacobian = std_msgs.msg.Float64MultiArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Float64MultiArray.getMessageSize(object.jacobian);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rbdl_server/RBDLJacobianResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d411acbcc9bdcb049d050b7d07331de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64MultiArray jacobian
    
    ================================================================================
    MSG: std_msgs/Float64MultiArray
    # Please look at the MultiArrayLayout message definition for
    # documentation on all multiarrays.
    
    MultiArrayLayout  layout        # specification of data layout
    float64[]         data          # array of data
    
    
    ================================================================================
    MSG: std_msgs/MultiArrayLayout
    # The multiarray declares a generic multi-dimensional array of a
    # particular data type.  Dimensions are ordered from outer most
    # to inner most.
    
    MultiArrayDimension[] dim # Array of dimension properties
    uint32 data_offset        # padding elements at front of data
    
    # Accessors should ALWAYS be written in terms of dimension stride
    # and specified outer-most dimension first.
    # 
    # multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    #
    # A standard, 3-channel 640x480 image with interleaved color channels
    # would be specified as:
    #
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
    # dim[2].size   = 3
    # dim[2].stride = 3
    #
    # multiarray(i,j,k) refers to the ith row, jth column, and kth channel.
    
    ================================================================================
    MSG: std_msgs/MultiArrayDimension
    string label   # label of given dimension
    uint32 size    # size of given dimension (in type units)
    uint32 stride  # stride of given dimension
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RBDLJacobianResponse(null);
    if (msg.jacobian !== undefined) {
      resolved.jacobian = std_msgs.msg.Float64MultiArray.Resolve(msg.jacobian)
    }
    else {
      resolved.jacobian = new std_msgs.msg.Float64MultiArray()
    }

    return resolved;
    }
};

module.exports = {
  Request: RBDLJacobianRequest,
  Response: RBDLJacobianResponse,
  md5sum() { return '4b2ed355812dee1bac3ae1b7e2f6c059'; },
  datatype() { return 'rbdl_server/RBDLJacobian'; }
};
