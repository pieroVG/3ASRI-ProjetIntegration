// Auto-generated. Do not edit!

// (in-package motoman_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DynamicJointsGroup {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.group_number = null;
      this.num_joints = null;
      this.valid_fields = null;
      this.positions = null;
      this.velocities = null;
      this.accelerations = null;
      this.effort = null;
      this.time_from_start = null;
    }
    else {
      if (initObj.hasOwnProperty('group_number')) {
        this.group_number = initObj.group_number
      }
      else {
        this.group_number = 0;
      }
      if (initObj.hasOwnProperty('num_joints')) {
        this.num_joints = initObj.num_joints
      }
      else {
        this.num_joints = 0;
      }
      if (initObj.hasOwnProperty('valid_fields')) {
        this.valid_fields = initObj.valid_fields
      }
      else {
        this.valid_fields = 0;
      }
      if (initObj.hasOwnProperty('positions')) {
        this.positions = initObj.positions
      }
      else {
        this.positions = [];
      }
      if (initObj.hasOwnProperty('velocities')) {
        this.velocities = initObj.velocities
      }
      else {
        this.velocities = [];
      }
      if (initObj.hasOwnProperty('accelerations')) {
        this.accelerations = initObj.accelerations
      }
      else {
        this.accelerations = [];
      }
      if (initObj.hasOwnProperty('effort')) {
        this.effort = initObj.effort
      }
      else {
        this.effort = [];
      }
      if (initObj.hasOwnProperty('time_from_start')) {
        this.time_from_start = initObj.time_from_start
      }
      else {
        this.time_from_start = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DynamicJointsGroup
    // Serialize message field [group_number]
    bufferOffset = _serializer.int16(obj.group_number, buffer, bufferOffset);
    // Serialize message field [num_joints]
    bufferOffset = _serializer.int16(obj.num_joints, buffer, bufferOffset);
    // Serialize message field [valid_fields]
    bufferOffset = _serializer.int16(obj.valid_fields, buffer, bufferOffset);
    // Serialize message field [positions]
    bufferOffset = _arraySerializer.float64(obj.positions, buffer, bufferOffset, null);
    // Serialize message field [velocities]
    bufferOffset = _arraySerializer.float64(obj.velocities, buffer, bufferOffset, null);
    // Serialize message field [accelerations]
    bufferOffset = _arraySerializer.float64(obj.accelerations, buffer, bufferOffset, null);
    // Serialize message field [effort]
    bufferOffset = _arraySerializer.float64(obj.effort, buffer, bufferOffset, null);
    // Serialize message field [time_from_start]
    bufferOffset = _serializer.duration(obj.time_from_start, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DynamicJointsGroup
    let len;
    let data = new DynamicJointsGroup(null);
    // Deserialize message field [group_number]
    data.group_number = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [num_joints]
    data.num_joints = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [valid_fields]
    data.valid_fields = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [positions]
    data.positions = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [velocities]
    data.velocities = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [accelerations]
    data.accelerations = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [effort]
    data.effort = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [time_from_start]
    data.time_from_start = _deserializer.duration(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.positions.length;
    length += 8 * object.velocities.length;
    length += 8 * object.accelerations.length;
    length += 8 * object.effort.length;
    return length + 30;
  }

  static datatype() {
    // Returns string type for a message object
    return 'motoman_msgs/DynamicJointsGroup';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fb56acaba1b90a9d7640af3e785681ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new DynamicJointsGroup(null);
    if (msg.group_number !== undefined) {
      resolved.group_number = msg.group_number;
    }
    else {
      resolved.group_number = 0
    }

    if (msg.num_joints !== undefined) {
      resolved.num_joints = msg.num_joints;
    }
    else {
      resolved.num_joints = 0
    }

    if (msg.valid_fields !== undefined) {
      resolved.valid_fields = msg.valid_fields;
    }
    else {
      resolved.valid_fields = 0
    }

    if (msg.positions !== undefined) {
      resolved.positions = msg.positions;
    }
    else {
      resolved.positions = []
    }

    if (msg.velocities !== undefined) {
      resolved.velocities = msg.velocities;
    }
    else {
      resolved.velocities = []
    }

    if (msg.accelerations !== undefined) {
      resolved.accelerations = msg.accelerations;
    }
    else {
      resolved.accelerations = []
    }

    if (msg.effort !== undefined) {
      resolved.effort = msg.effort;
    }
    else {
      resolved.effort = []
    }

    if (msg.time_from_start !== undefined) {
      resolved.time_from_start = msg.time_from_start;
    }
    else {
      resolved.time_from_start = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = DynamicJointsGroup;
