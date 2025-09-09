// Auto-generated. Do not edit!

// (in-package motoman_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ReadMRegisterRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.address = null;
    }
    else {
      if (initObj.hasOwnProperty('address')) {
        this.address = initObj.address
      }
      else {
        this.address = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReadMRegisterRequest
    // Serialize message field [address]
    bufferOffset = _serializer.uint32(obj.address, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadMRegisterRequest
    let len;
    let data = new ReadMRegisterRequest(null);
    // Deserialize message field [address]
    data.address = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motoman_msgs/ReadMRegisterRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7522928648cae826d000156b5f561c00';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # Read (and return) the current value of the M register at 'address'.
    #
    # Addresses are plain, base-10 integers, as used and displayed by the controller
    # (on the teach pendant for instance).
    #
    # Only the following addresses can be read from:
    #
    #  - 0 to 999
    #
    # NOTE 1: do not add 1000000 to the address, MotoROS will do this when
    #         necessary.
    #
    # NOTE 2: many programming languages will parse literals starting with '0' as
    #         octal numbers. Do not add leading zeros to register addresses to avoid
    #         specifying the wrong register to read.
    #
    # Refer also the Yaskawa Motoman documentation on IO addressing and
    # configuration.
    
    uint32 address
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReadMRegisterRequest(null);
    if (msg.address !== undefined) {
      resolved.address = msg.address;
    }
    else {
      resolved.address = 0
    }

    return resolved;
    }
};

class ReadMRegisterResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.message = null;
      this.success = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReadMRegisterResponse
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.uint16(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadMRegisterResponse
    let len;
    let data = new ReadMRegisterResponse(null);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 7;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motoman_msgs/ReadMRegisterResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a0305ccc5f57b472aa935ba809c1b3ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string message
    bool success
    uint16 value
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReadMRegisterResponse(null);
    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: ReadMRegisterRequest,
  Response: ReadMRegisterResponse,
  md5sum() { return 'a3d35ac86126c0934861b8c43f69c8b8'; },
  datatype() { return 'motoman_msgs/ReadMRegister'; }
};
