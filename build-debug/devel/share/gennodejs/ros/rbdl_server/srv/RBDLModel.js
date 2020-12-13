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

class RBDLModelRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.model = null;
    }
    else {
      if (initObj.hasOwnProperty('model')) {
        this.model = initObj.model
      }
      else {
        this.model = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RBDLModelRequest
    // Serialize message field [model]
    bufferOffset = _serializer.string(obj.model, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RBDLModelRequest
    let len;
    let data = new RBDLModelRequest(null);
    // Deserialize message field [model]
    data.model = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.model.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rbdl_server/RBDLModelRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0147e4f36cba5cda7fa39c089e493413';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string model
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RBDLModelRequest(null);
    if (msg.model !== undefined) {
      resolved.model = msg.model;
    }
    else {
      resolved.model = ''
    }

    return resolved;
    }
};

class RBDLModelResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.good = null;
    }
    else {
      if (initObj.hasOwnProperty('good')) {
        this.good = initObj.good
      }
      else {
        this.good = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RBDLModelResponse
    // Serialize message field [good]
    bufferOffset = _serializer.bool(obj.good, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RBDLModelResponse
    let len;
    let data = new RBDLModelResponse(null);
    // Deserialize message field [good]
    data.good = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rbdl_server/RBDLModelResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '420aeb7337f0734e9777121e7c6c96a3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool good
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RBDLModelResponse(null);
    if (msg.good !== undefined) {
      resolved.good = msg.good;
    }
    else {
      resolved.good = false
    }

    return resolved;
    }
};

module.exports = {
  Request: RBDLModelRequest,
  Response: RBDLModelResponse,
  md5sum() { return 'd092f66b4213534ea05b76cd5ee71314'; },
  datatype() { return 'rbdl_server/RBDLModel'; }
};
