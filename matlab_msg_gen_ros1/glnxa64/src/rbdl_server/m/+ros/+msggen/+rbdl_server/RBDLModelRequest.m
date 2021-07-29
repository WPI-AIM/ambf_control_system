
classdef RBDLModelRequest < ros.Message
    %RBDLModelRequest MATLAB implementation of rbdl_server/RBDLModelRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rbdl_server/RBDLModelRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'a2aa2b812fc1ca31173f5b08285e210f' % The MD5 Checksum of the message definition
        PropertyList = { 'ModelName' 'Model' } % List of non-constant message properties
        ROSPropertyList = { 'model_name' 'model' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ModelName
        Model
    end
    methods
        function set.ModelName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'RBDLModelRequest', 'ModelName');
            obj.ModelName = char(val);
        end
        function set.Model(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'RBDLModelRequest', 'Model');
            obj.Model = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rbdl_server.RBDLModelRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rbdl_server.RBDLModelRequest(strObj);
        end
    end
end