// Auto-generated. Do not edit!

// (in-package motoman_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DynamicJointTrajectory = require('../msg/DynamicJointTrajectory.js');

//-----------------------------------------------------------

let industrial_msgs = _finder('industrial_msgs');

//-----------------------------------------------------------

class CmdJointTrajectoryExRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.trajectory = null;
    }
    else {
      if (initObj.hasOwnProperty('trajectory')) {
        this.trajectory = initObj.trajectory
      }
      else {
        this.trajectory = new DynamicJointTrajectory();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CmdJointTrajectoryExRequest
    // Serialize message field [trajectory]
    bufferOffset = DynamicJointTrajectory.serialize(obj.trajectory, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CmdJointTrajectoryExRequest
    let len;
    let data = new CmdJointTrajectoryExRequest(null);
    // Deserialize message field [trajectory]
    data.trajectory = DynamicJointTrajectory.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += DynamicJointTrajectory.getMessageSize(object.trajectory);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motoman_msgs/CmdJointTrajectoryExRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7896a81fd909fb14239085c546790e08';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    motoman_msgs/DynamicJointTrajectory trajectory
    
    ================================================================================
    MSG: motoman_msgs/DynamicJointTrajectory
    #length: true message/data length
    #header: 
    #sequence:
    #num_groups: # of motion groups included in this message
    #group[]: DynamicJointPoint from DynamicJointPoint.msg
    
    Header header
    string[] joint_names
    DynamicJointPoint[] points
    
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
    MSG: motoman_msgs/DynamicJointPoint
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
    const resolved = new CmdJointTrajectoryExRequest(null);
    if (msg.trajectory !== undefined) {
      resolved.trajectory = DynamicJointTrajectory.Resolve(msg.trajectory)
    }
    else {
      resolved.trajectory = new DynamicJointTrajectory()
    }

    return resolved;
    }
};

class CmdJointTrajectoryExResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.code = null;
    }
    else {
      if (initObj.hasOwnProperty('code')) {
        this.code = initObj.code
      }
      else {
        this.code = new industrial_msgs.msg.ServiceReturnCode();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CmdJointTrajectoryExResponse
    // Serialize message field [code]
    bufferOffset = industrial_msgs.msg.ServiceReturnCode.serialize(obj.code, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CmdJointTrajectoryExResponse
    let len;
    let data = new CmdJointTrajectoryExResponse(null);
    // Deserialize message field [code]
    data.code = industrial_msgs.msg.ServiceReturnCode.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motoman_msgs/CmdJointTrajectoryExResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '50b1f38f75f5677e5692f3b3e7e1ea48';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    industrial_msgs/ServiceReturnCode code
    
    
    ================================================================================
    MSG: industrial_msgs/ServiceReturnCode
    # Service return codes for simple requests.  All ROS-Industrial service
    # replies are required to have a return code indicating success or failure
    # Specific return codes for different failure should be negative.
    int8 val
    
    int8 SUCCESS = 1
    int8 FAILURE = -1
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CmdJointTrajectoryExResponse(null);
    if (msg.code !== undefined) {
      resolved.code = industrial_msgs.msg.ServiceReturnCode.Resolve(msg.code)
    }
    else {
      resolved.code = new industrial_msgs.msg.ServiceReturnCode()
    }

    return resolved;
    }
};

module.exports = {
  Request: CmdJointTrajectoryExRequest,
  Response: CmdJointTrajectoryExResponse,
  md5sum() { return '2c13063e3bd4783949860a7cb1a82ea9'; },
  datatype() { return 'motoman_msgs/CmdJointTrajectoryEx'; }
};
