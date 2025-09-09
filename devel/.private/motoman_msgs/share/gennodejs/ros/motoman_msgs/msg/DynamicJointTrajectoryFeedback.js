// Auto-generated. Do not edit!

// (in-package motoman_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DynamicJointState = require('./DynamicJointState.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DynamicJointTrajectoryFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.num_groups = null;
      this.joint_feedbacks = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('num_groups')) {
        this.num_groups = initObj.num_groups
      }
      else {
        this.num_groups = 0;
      }
      if (initObj.hasOwnProperty('joint_feedbacks')) {
        this.joint_feedbacks = initObj.joint_feedbacks
      }
      else {
        this.joint_feedbacks = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DynamicJointTrajectoryFeedback
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [num_groups]
    bufferOffset = _serializer.int16(obj.num_groups, buffer, bufferOffset);
    // Serialize message field [joint_feedbacks]
    // Serialize the length for message field [joint_feedbacks]
    bufferOffset = _serializer.uint32(obj.joint_feedbacks.length, buffer, bufferOffset);
    obj.joint_feedbacks.forEach((val) => {
      bufferOffset = DynamicJointState.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DynamicJointTrajectoryFeedback
    let len;
    let data = new DynamicJointTrajectoryFeedback(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_groups]
    data.num_groups = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [joint_feedbacks]
    // Deserialize array length for message field [joint_feedbacks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.joint_feedbacks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.joint_feedbacks[i] = DynamicJointState.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.joint_feedbacks.forEach((val) => {
      length += DynamicJointState.getMessageSize(val);
    });
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'motoman_msgs/DynamicJointTrajectoryFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '84d3bbf7103790ff0a8946017b895a1a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int16 num_groups
    DynamicJointState[] joint_feedbacks
    
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
    MSG: motoman_msgs/DynamicJointState
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
    const resolved = new DynamicJointTrajectoryFeedback(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.num_groups !== undefined) {
      resolved.num_groups = msg.num_groups;
    }
    else {
      resolved.num_groups = 0
    }

    if (msg.joint_feedbacks !== undefined) {
      resolved.joint_feedbacks = new Array(msg.joint_feedbacks.length);
      for (let i = 0; i < resolved.joint_feedbacks.length; ++i) {
        resolved.joint_feedbacks[i] = DynamicJointState.Resolve(msg.joint_feedbacks[i]);
      }
    }
    else {
      resolved.joint_feedbacks = []
    }

    return resolved;
    }
};

module.exports = DynamicJointTrajectoryFeedback;
