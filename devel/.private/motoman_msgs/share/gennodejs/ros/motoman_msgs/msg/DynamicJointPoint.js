// Auto-generated. Do not edit!

// (in-package motoman_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DynamicJointsGroup = require('./DynamicJointsGroup.js');

//-----------------------------------------------------------

class DynamicJointPoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num_groups = null;
      this.groups = null;
    }
    else {
      if (initObj.hasOwnProperty('num_groups')) {
        this.num_groups = initObj.num_groups
      }
      else {
        this.num_groups = 0;
      }
      if (initObj.hasOwnProperty('groups')) {
        this.groups = initObj.groups
      }
      else {
        this.groups = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DynamicJointPoint
    // Serialize message field [num_groups]
    bufferOffset = _serializer.int16(obj.num_groups, buffer, bufferOffset);
    // Serialize message field [groups]
    // Serialize the length for message field [groups]
    bufferOffset = _serializer.uint32(obj.groups.length, buffer, bufferOffset);
    obj.groups.forEach((val) => {
      bufferOffset = DynamicJointsGroup.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DynamicJointPoint
    let len;
    let data = new DynamicJointPoint(null);
    // Deserialize message field [num_groups]
    data.num_groups = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [groups]
    // Deserialize array length for message field [groups]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.groups = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.groups[i] = DynamicJointsGroup.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.groups.forEach((val) => {
      length += DynamicJointsGroup.getMessageSize(val);
    });
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'motoman_msgs/DynamicJointPoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f91ca86c2821b55c8430ab0088bfe5df';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # DynamicJointPoint
    #group: # length of this array must match num_groups
    #    id:   control-group ID for use on-controller
    #    num_joints: # of joints in this motion group
    #    valid_fields: #bit field for following items
    #    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
    #    positions[]
    #    velocities[]
    #    accelerations[]
    #    effort[]
    #    time_from_start
    
    int16 num_groups
    DynamicJointsGroup[] groups
    
    ================================================================================
    MSG: motoman_msgs/DynamicJointsGroup
    # DynamicJointsGroup
    #group: # length of this array must match num_groups
    #    id:   control-group ID for use on-controller
    #    num_joints: # of joints in this motion group
    #    valid_fields: #bit field for following items
    #    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
    #    positions[]
    #    velocities[]
    #    accelerations[]
    #    effort[]
    #    time_from_start
    
    
    int16 group_number
    int16 num_joints
    int16 valid_fields
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
    const resolved = new DynamicJointPoint(null);
    if (msg.num_groups !== undefined) {
      resolved.num_groups = msg.num_groups;
    }
    else {
      resolved.num_groups = 0
    }

    if (msg.groups !== undefined) {
      resolved.groups = new Array(msg.groups.length);
      for (let i = 0; i < resolved.groups.length; ++i) {
        resolved.groups[i] = DynamicJointsGroup.Resolve(msg.groups[i]);
      }
    }
    else {
      resolved.groups = []
    }

    return resolved;
    }
};

module.exports = DynamicJointPoint;
