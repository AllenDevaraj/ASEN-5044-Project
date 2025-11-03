// Auto-generated. Do not edit!

// (in-package pick_and_place.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DetectedObject {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_world = null;
      this.y_world = null;
      this.z_world = null;
      this.width = null;
      this.length = null;
      this.height = null;
      this.color = null;
    }
    else {
      if (initObj.hasOwnProperty('x_world')) {
        this.x_world = initObj.x_world
      }
      else {
        this.x_world = 0.0;
      }
      if (initObj.hasOwnProperty('y_world')) {
        this.y_world = initObj.y_world
      }
      else {
        this.y_world = 0.0;
      }
      if (initObj.hasOwnProperty('z_world')) {
        this.z_world = initObj.z_world
      }
      else {
        this.z_world = 0.0;
      }
      if (initObj.hasOwnProperty('width')) {
        this.width = initObj.width
      }
      else {
        this.width = 0.0;
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0.0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0.0;
      }
      if (initObj.hasOwnProperty('color')) {
        this.color = initObj.color
      }
      else {
        this.color = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DetectedObject
    // Serialize message field [x_world]
    bufferOffset = _serializer.float64(obj.x_world, buffer, bufferOffset);
    // Serialize message field [y_world]
    bufferOffset = _serializer.float64(obj.y_world, buffer, bufferOffset);
    // Serialize message field [z_world]
    bufferOffset = _serializer.float64(obj.z_world, buffer, bufferOffset);
    // Serialize message field [width]
    bufferOffset = _serializer.float64(obj.width, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.float64(obj.length, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.float64(obj.height, buffer, bufferOffset);
    // Serialize message field [color]
    bufferOffset = _serializer.string(obj.color, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DetectedObject
    let len;
    let data = new DetectedObject(null);
    // Deserialize message field [x_world]
    data.x_world = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_world]
    data.y_world = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z_world]
    data.z_world = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [width]
    data.width = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [color]
    data.color = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.color);
    return length + 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pick_and_place/DetectedObject';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '167a85e1fd80883aabe7acad7edd6a85';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x_world
    float64 y_world
    float64 z_world
    float64 width
    float64 length
    float64 height
    string color
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DetectedObject(null);
    if (msg.x_world !== undefined) {
      resolved.x_world = msg.x_world;
    }
    else {
      resolved.x_world = 0.0
    }

    if (msg.y_world !== undefined) {
      resolved.y_world = msg.y_world;
    }
    else {
      resolved.y_world = 0.0
    }

    if (msg.z_world !== undefined) {
      resolved.z_world = msg.z_world;
    }
    else {
      resolved.z_world = 0.0
    }

    if (msg.width !== undefined) {
      resolved.width = msg.width;
    }
    else {
      resolved.width = 0.0
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0.0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0.0
    }

    if (msg.color !== undefined) {
      resolved.color = msg.color;
    }
    else {
      resolved.color = ''
    }

    return resolved;
    }
};

module.exports = DetectedObject;
