// Auto-generated. Do not edit!

// (in-package rbdl_server.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class RBDLForwardDynamicsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.q = null;
      this.qd = null;
      this.tau = null;
    }
    else {
      if (initObj.hasOwnProperty('q')) {
        this.q = initObj.q
      }
      else {
        this.q = [];
      }
      if (initObj.hasOwnProperty('qd')) {
        this.qd = initObj.qd
      }
      else {
        this.qd = [];
      }
      if (initObj.hasOwnProperty('tau')) {
        this.tau = initObj.tau
      }
      else {
        this.tau = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RBDLForwardDynamicsRequest
    // Serialize message field [q]
    bufferOffset = _arraySerializer.float64(obj.q, buffer, bufferOffset, null);
    // Serialize message field [qd]
    bufferOffset = _arraySerializer.float64(obj.qd, buffer, bufferOffset, null);
    // Serialize message field [tau]
    bufferOffset = _arraySerializer.float64(obj.tau, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RBDLForwardDynamicsRequest
    let len;
    let data = new RBDLForwardDynamicsRequest(null);
    // Deserialize message field [q]
    data.q = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [qd]
    data.qd = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [tau]
    data.tau = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.q.length;
    length += 8 * object.qd.length;
    length += 8 * object.tau.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rbdl_server/RBDLForwardDynamicsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '65aaad731b9018a695899e0b9a800eeb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] q
    float64[] qd
    float64[] tau
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RBDLForwardDynamicsRequest(null);
    if (msg.q !== undefined) {
      resolved.q = msg.q;
    }
    else {
      resolved.q = []
    }

    if (msg.qd !== undefined) {
      resolved.qd = msg.qd;
    }
    else {
      resolved.qd = []
    }

    if (msg.tau !== undefined) {
      resolved.tau = msg.tau;
    }
    else {
      resolved.tau = []
    }

    return resolved;
    }
};

class RBDLForwardDynamicsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.qdd = null;
    }
    else {
      if (initObj.hasOwnProperty('qdd')) {
        this.qdd = initObj.qdd
      }
      else {
        this.qdd = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RBDLForwardDynamicsResponse
    // Serialize message field [qdd]
    bufferOffset = _arraySerializer.float64(obj.qdd, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RBDLForwardDynamicsResponse
    let len;
    let data = new RBDLForwardDynamicsResponse(null);
    // Deserialize message field [qdd]
    data.qdd = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.qdd.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rbdl_server/RBDLForwardDynamicsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f9f5346aad9a21a7c1a3fdc3006a6c17';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] qdd
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RBDLForwardDynamicsResponse(null);
    if (msg.qdd !== undefined) {
      resolved.qdd = msg.qdd;
    }
    else {
      resolved.qdd = []
    }

    return resolved;
    }
};

module.exports = {
  Request: RBDLForwardDynamicsRequest,
  Response: RBDLForwardDynamicsResponse,
  md5sum() { return '2510e5eefce24dbe51d450b7f1432820'; },
  datatype() { return 'rbdl_server/RBDLForwardDynamics'; }
};
