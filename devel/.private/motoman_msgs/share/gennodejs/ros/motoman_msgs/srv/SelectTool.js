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

class SelectToolRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.group_number = null;
      this.tool_number = null;
    }
    else {
      if (initObj.hasOwnProperty('group_number')) {
        this.group_number = initObj.group_number
      }
      else {
        this.group_number = 0;
      }
      if (initObj.hasOwnProperty('tool_number')) {
        this.tool_number = initObj.tool_number
      }
      else {
        this.tool_number = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SelectToolRequest
    // Serialize message field [group_number]
    bufferOffset = _serializer.uint32(obj.group_number, buffer, bufferOffset);
    // Serialize message field [tool_number]
    bufferOffset = _serializer.uint32(obj.tool_number, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SelectToolRequest
    let len;
    let data = new SelectToolRequest(null);
    // Deserialize message field [group_number]
    data.group_number = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [tool_number]
    data.tool_number = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motoman_msgs/SelectToolRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fa612b76af0dfd1addb4c87693871b10';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # Change the active tool file on the controller.
    #
    # This changes the tool definition used for both (Moto)ROS-controlled motions
    # and manual jogging.
    #
    #
    # ## Controller support
    #
    # This service wraps two distinct (but related) actions:
    #
    #  1. changing the tool file used for execution of (Moto)ROS-controlled motions
    #  2. changing the tool file used for jogging
    #
    # Action 1 is supported by all controllers supported by MotoROS (ie: DX100,
    # FS100, DX200 and YRC1000(u)).
    #
    # Action 2 is NOT supported on FS100 controllers. This is a limitation of
    # MotoPlus, not of MotoROS.
    #
    # In all cases, make sure to read the following sections carefully to be aware
    # of the behaviour of this service and of the Yaskawa controller after calling
    # this service.
    #
    #
    # ## FSU and PFL
    #
    # Tool switches through this service will be taken into account by the FSU and
    # PFL safety systems if the controller has these options and they are enabled.
    #
    # Tool interference files, if associated with certain tool file definitions,
    # will therefore also switch.
    #
    #
    # ## Tool switch timing
    #
    # The active tool will be switched AFTER the controller has completed execution
    # of the last trajectory segment that was sent to MotoROS BEFORE this service
    # was called.
    #
    # The motion queue internal to MotoROS caches a number of segments in a FIFO.
    # Only segments received AFTER this service was invoked will be executed with
    # the new tool. Any segments received before a tool switch was invoked will use
    # the old tool.
    #
    # To ensure motion will be executed using a certain tool, applications should
    # check the 'in_motion' field (part of the RobotStatus message published on the
    # 'robot_status' topic by the driver) and invoke the service when the robot has
    # come to a stop (and the motion queue of MotoROS is empty). Any subsequent
    # trajectories will use the new tool.
    #
    #
    # ## Effect on trajectory execution
    #
    # As MotoROS currently only executes joint space trajectories, changing tool
    # file with this service DOES NOT affect the execution of those trajectories.
    #
    # As noted earlier though, the active tool file definition will affect FSU and
    # PFL behaviour, as the tool definition is used in calculation of dynamics and
    # safety (see "FSU and PFL" above).
    #
    # To clarify: the TCP definition of the tool file is NOT taken into account when
    # calculating manipulator motion based on incoming ROS JointTrajectoryAction
    # goals (as JointTrajectory goals do not include any Cartesian poses, only
    # absolute joint space coordinates for each axis).
    #
    # Instead, ROS applications should use different TF frames to define tool frames
    # on the ROS side and plan their motions with the appropriate TF frame as the
    # active tool.
    #
    # This service could then be used to notify the controller of other tool
    # characteristics, such as weight, CoG and inertia by switching to a tool file
    # configured with those parameters.
    #
    #
    # ## Retrieving the active tool file
    #
    # MotoROS does not currently support retrieving the active tool file.
    #
    #
    #
    # For more information about tool file configuration, selection and use on
    # Yaskawa controllers, refer to the relevant Yaskawa Motoman documentation.
    
    
    # Numeric ID of the group the tool file is defined for.
    #
    # This MUST correspond to a group ID currently defined and active on the
    # controller. The ROS service does not check whether the value specified here
    # is legal. The MotoROS server will however check this, and will refuse to
    # switch to an illegal value and will report this to the ROS driver.
    #
    # Callers will be notified of such failures by 'success' being set to 'false'
    # and an appropriate error message being provided via the 'message' field of
    # the service response.
    #
    # NOTE: this field is 0-based, with 0 corresponding to the first motion group,
    #       1 the second, etc.
    #
    # legal-values: [0, total_nr_of_groups)
    # required: yes (absence-causes-service-failure)
    # default: no-default
    uint32 group_number
    
    # Numeric ID of the tool file to switch to for the specified group.
    #
    # Identical to 'group_number', specification of illegal values will result
    # in failure to invoke the MotoROS service, and an unsuccessful service result
    # being returned.
    #
    # NOTE: this field is 0-based, with 0 corresponding to the first tool file,
    #       1 the second, etc.
    #
    # legal-values: [0, 63]
    # required: yes (absence-causes-service-failure)
    # default: no-default
    uint32 tool_number
    
    # TODO: might want to expose 'sequence_number' here as well
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SelectToolRequest(null);
    if (msg.group_number !== undefined) {
      resolved.group_number = msg.group_number;
    }
    else {
      resolved.group_number = 0
    }

    if (msg.tool_number !== undefined) {
      resolved.tool_number = msg.tool_number;
    }
    else {
      resolved.tool_number = 0
    }

    return resolved;
    }
};

class SelectToolResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.message = null;
      this.success = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SelectToolResponse
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SelectToolResponse
    let len;
    let data = new SelectToolResponse(null);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motoman_msgs/SelectToolResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9bf829f07d795d3f9e541a07897da2c4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # A human-readable error message, or an empty string if there was no error.
    string message
    
    # true IFF invocation of the MotoROS service was successful.
    #
    # NOTE: this is independent of whether the ROS service invocation was
    #       successful. In absence of any ROS middleware failures, a failed MotoROS
    #       service invocation will result in 'success' here being set to 'false',
    #       but a successful ROS service invocation.
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SelectToolResponse(null);
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

    return resolved;
    }
};

module.exports = {
  Request: SelectToolRequest,
  Response: SelectToolResponse,
  md5sum() { return 'afb2fb884fcd9c28c81c82056d543156'; },
  datatype() { return 'motoman_msgs/SelectTool'; }
};
