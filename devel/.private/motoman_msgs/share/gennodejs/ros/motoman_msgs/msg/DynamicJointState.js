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

class DynamicJointState {
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
      this.positions_desired = null;
      this.positions_errors = null;
      this.velocities_desired = null;
      this.velocities_errors = null;
      this.accelerations_desired = null;
      this.accelerations_errors = null;
      this.effort_errors = null;
      this.effort_desired = null;
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
      if (initObj.hasOwnProperty('positions_desired')) {
        this.positions_desired = initObj.positions_desired
      }
      else {
        this.positions_desired = [];
      }
      if (initObj.hasOwnProperty('positions_errors')) {
        this.positions_errors = initObj.positions_errors
      }
      else {
        this.positions_errors = [];
      }
      if (initObj.hasOwnProperty('velocities_desired')) {
        this.velocities_desired = initObj.velocities_desired
      }
      else {
        this.velocities_desired = [];
      }
      if (initObj.hasOwnProperty('velocities_errors')) {
        this.velocities_errors = initObj.velocities_errors
      }
      else {
        this.velocities_errors = [];
      }
      if (initObj.hasOwnProperty('accelerations_desired')) {
        this.accelerations_desired = initObj.accelerations_desired
      }
      else {
        this.accelerations_desired = [];
      }
      if (initObj.hasOwnProperty('accelerations_errors')) {
        this.accelerations_errors = initObj.accelerations_errors
      }
      else {
        this.accelerations_errors = [];
      }
      if (initObj.hasOwnProperty('effort_errors')) {
        this.effort_errors = initObj.effort_errors
      }
      else {
        this.effort_errors = [];
      }
      if (initObj.hasOwnProperty('effort_desired')) {
        this.effort_desired = initObj.effort_desired
      }
      else {
        this.effort_desired = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DynamicJointState
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
    // Serialize message field [positions_desired]
    bufferOffset = _arraySerializer.float64(obj.positions_desired, buffer, bufferOffset, null);
    // Serialize message field [positions_errors]
    bufferOffset = _arraySerializer.float64(obj.positions_errors, buffer, bufferOffset, null);
    // Serialize message field [velocities_desired]
    bufferOffset = _arraySerializer.float64(obj.velocities_desired, buffer, bufferOffset, null);
    // Serialize message field [velocities_errors]
    bufferOffset = _arraySerializer.float64(obj.velocities_errors, buffer, bufferOffset, null);
    // Serialize message field [accelerations_desired]
    bufferOffset = _arraySerializer.float64(obj.accelerations_desired, buffer, bufferOffset, null);
    // Serialize message field [accelerations_errors]
    bufferOffset = _arraySerializer.float64(obj.accelerations_errors, buffer, bufferOffset, null);
    // Serialize message field [effort_errors]
    bufferOffset = _arraySerializer.float64(obj.effort_errors, buffer, bufferOffset, null);
    // Serialize message field [effort_desired]
    bufferOffset = _arraySerializer.float64(obj.effort_desired, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DynamicJointState
    let len;
    let data = new DynamicJointState(null);
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
    // Deserialize message field [positions_desired]
    data.positions_desired = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [positions_errors]
    data.positions_errors = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [velocities_desired]
    data.velocities_desired = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [velocities_errors]
    data.velocities_errors = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [accelerations_desired]
    data.accelerations_desired = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [accelerations_errors]
    data.accelerations_errors = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [effort_errors]
    data.effort_errors = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [effort_desired]
    data.effort_desired = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.positions.length;
    length += 8 * object.velocities.length;
    length += 8 * object.accelerations.length;
    length += 8 * object.effort.length;
    length += 8 * object.positions_desired.length;
    length += 8 * object.positions_errors.length;
    length += 8 * object.velocities_desired.length;
    length += 8 * object.velocities_errors.length;
    length += 8 * object.accelerations_desired.length;
    length += 8 * object.accelerations_errors.length;
    length += 8 * object.effort_errors.length;
    length += 8 * object.effort_desired.length;
    return length + 54;
  }

  static datatype() {
    // Returns string type for a message object
    return 'motoman_msgs/DynamicJointState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c44649b8de969b98f15adea419fa49a4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #group[]: # length of this array must match num_groups
    #    id:   control-group ID for use on-controller
    #    num_joints: # of joints in this motion group
    #    valid_fields: #bit field for following items
    #    # length of the following items must match num_joints, order set by controller.  Invalid fields (see bit field above) are not included, resulting in a shorter message.
    #    positions[]
    #    velocities[]
    #    accelerations[]
    #    effort[]
    #    position_desired[]
    #    position_errors[]
    #    velocity_desired[]
    #    velocity_errors[]
    #    effort_desired[]
    #    effort_error[]
    
    int16 group_number
    int16 num_joints
    int16 valid_fields
    float64[] positions
    float64[] velocities
    float64[] accelerations
    float64[] effort
    float64[] positions_desired
    float64[] positions_errors
    float64[] velocities_desired
    float64[] velocities_errors
    float64[] accelerations_desired
    float64[] accelerations_errors
    float64[] effort_errors
    float64[] effort_desired
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DynamicJointState(null);
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

    if (msg.positions_desired !== undefined) {
      resolved.positions_desired = msg.positions_desired;
    }
    else {
      resolved.positions_desired = []
    }

    if (msg.positions_errors !== undefined) {
      resolved.positions_errors = msg.positions_errors;
    }
    else {
      resolved.positions_errors = []
    }

    if (msg.velocities_desired !== undefined) {
      resolved.velocities_desired = msg.velocities_desired;
    }
    else {
      resolved.velocities_desired = []
    }

    if (msg.velocities_errors !== undefined) {
      resolved.velocities_errors = msg.velocities_errors;
    }
    else {
      resolved.velocities_errors = []
    }

    if (msg.accelerations_desired !== undefined) {
      resolved.accelerations_desired = msg.accelerations_desired;
    }
    else {
      resolved.accelerations_desired = []
    }

    if (msg.accelerations_errors !== undefined) {
      resolved.accelerations_errors = msg.accelerations_errors;
    }
    else {
      resolved.accelerations_errors = []
    }

    if (msg.effort_errors !== undefined) {
      resolved.effort_errors = msg.effort_errors;
    }
    else {
      resolved.effort_errors = []
    }

    if (msg.effort_desired !== undefined) {
      resolved.effort_desired = msg.effort_desired;
    }
    else {
      resolved.effort_desired = []
    }

    return resolved;
    }
};

module.exports = DynamicJointState;
