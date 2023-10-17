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

class ActionnersMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.act = null;
      this.fail = null;
      this.end = null;
    }
    else {
      if (initObj.hasOwnProperty('act')) {
        this.act = initObj.act
      }
      else {
        this.act = '';
      }
      if (initObj.hasOwnProperty('fail')) {
        this.fail = initObj.fail
      }
      else {
        this.fail = false;
      }
      if (initObj.hasOwnProperty('end')) {
        this.end = initObj.end
      }
      else {
        this.end = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActionnersMsg
    // Serialize message field [act]
    bufferOffset = _serializer.string(obj.act, buffer, bufferOffset);
    // Serialize message field [fail]
    bufferOffset = _serializer.bool(obj.fail, buffer, bufferOffset);
    // Serialize message field [end]
    bufferOffset = _serializer.bool(obj.end, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActionnersMsg
    let len;
    let data = new ActionnersMsg(null);
    // Deserialize message field [act]
    data.act = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [fail]
    data.fail = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [end]
    data.end = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.act);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'message/ActionnersMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3dd2e8d97f70009cf8e47cbf3e745186';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string act
    bool fail
    bool end
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActionnersMsg(null);
    if (msg.act !== undefined) {
      resolved.act = msg.act;
    }
    else {
      resolved.act = ''
    }

    if (msg.fail !== undefined) {
      resolved.fail = msg.fail;
    }
    else {
      resolved.fail = false
    }

    if (msg.end !== undefined) {
      resolved.end = msg.end;
    }
    else {
      resolved.end = false
    }

    return resolved;
    }
};

module.exports = ActionnersMsg;
