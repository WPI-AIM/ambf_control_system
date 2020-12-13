// Auto-generated. Do not edit!

// (in-package controller_modules.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ControllerListRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerListRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerListRequest
    let len;
    let data = new ControllerListRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'controller_modules/ControllerListRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerListRequest(null);
    return resolved;
    }
};

class ControllerListResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.controllers = null;
    }
    else {
      if (initObj.hasOwnProperty('controllers')) {
        this.controllers = initObj.controllers
      }
      else {
        this.controllers = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerListResponse
    // Serialize message field [controllers]
    bufferOffset = _arraySerializer.string(obj.controllers, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerListResponse
    let len;
    let data = new ControllerListResponse(null);
    // Deserialize message field [controllers]
    data.controllers = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.controllers.forEach((val) => {
      length += 4 + val.length;
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'controller_modules/ControllerListResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8242783a458107f102bce0800c333f0a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] controllers
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerListResponse(null);
    if (msg.controllers !== undefined) {
      resolved.controllers = msg.controllers;
    }
    else {
      resolved.controllers = []
    }

    return resolved;
    }
};

module.exports = {
  Request: ControllerListRequest,
  Response: ControllerListResponse,
  md5sum() { return '8242783a458107f102bce0800c333f0a'; },
  datatype() { return 'controller_modules/ControllerList'; }
};
