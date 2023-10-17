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

class EndOfActionMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.exit = null;
      this.reason = null;
    }
    else {
      if (initObj.hasOwnProperty('exit')) {
        this.exit = initObj.exit
      }
      else {
        this.exit = 0;
      }
      if (initObj.hasOwnProperty('reason')) {
        this.reason = initObj.reason
      }
      else {
        this.reason = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EndOfActionMsg
    // Serialize message field [exit]
    bufferOffset = _serializer.int16(obj.exit, buffer, bufferOffset);
    // Serialize message field [reason]
    bufferOffset = _serializer.string(obj.reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EndOfActionMsg
    let len;
    let data = new EndOfActionMsg(null);
    // Deserialize message field [exit]
    data.exit = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [reason]
    data.reason = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.reason);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'message/EndOfActionMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6620ca083d96e08d9ae896bc1472aad2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 exit
    string reason
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EndOfActionMsg(null);
    if (msg.exit !== undefined) {
      resolved.exit = msg.exit;
    }
    else {
      resolved.exit = 0
    }

    if (msg.reason !== undefined) {
      resolved.reason = msg.reason;
    }
    else {
      resolved.reason = ''
    }

    return resolved;
    }
};

module.exports = EndOfActionMsg;
