// Auto-generated. Do not edit!

// (in-package message.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class InfoMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.otherspos = null;
      this.possact = null;
      this.timeleft = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('otherspos')) {
        this.otherspos = initObj.otherspos
      }
      else {
        this.otherspos = [];
      }
      if (initObj.hasOwnProperty('possact')) {
        this.possact = initObj.possact
      }
      else {
        this.possact = [];
      }
      if (initObj.hasOwnProperty('timeleft')) {
        this.timeleft = initObj.timeleft
      }
      else {
        this.timeleft = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InfoMsg
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [otherspos]
    bufferOffset = _arraySerializer.float32(obj.otherspos, buffer, bufferOffset, null);
    // Serialize message field [possact]
    bufferOffset = _arraySerializer.int16(obj.possact, buffer, bufferOffset, null);
    // Serialize message field [timeleft]
    bufferOffset = _serializer.float32(obj.timeleft, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InfoMsg
    let len;
    let data = new InfoMsg(null);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [otherspos]
    data.otherspos = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [possact]
    data.possact = _arrayDeserializer.int16(buffer, bufferOffset, null)
    // Deserialize message field [timeleft]
    data.timeleft = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.otherspos.length;
    length += 2 * object.possact.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'message/InfoMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '74243b5379e1c0f977f139ace36782e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 x
    float32 y
    float32[] otherspos
    int16[] possact
    float32 timeleft
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InfoMsg(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.otherspos !== undefined) {
      resolved.otherspos = msg.otherspos;
    }
    else {
      resolved.otherspos = []
    }

    if (msg.possact !== undefined) {
      resolved.possact = msg.possact;
    }
    else {
      resolved.possact = []
    }

    if (msg.timeleft !== undefined) {
      resolved.timeleft = msg.timeleft;
    }
    else {
      resolved.timeleft = 0.0
    }

    return resolved;
    }
};

module.exports = InfoMsg;
