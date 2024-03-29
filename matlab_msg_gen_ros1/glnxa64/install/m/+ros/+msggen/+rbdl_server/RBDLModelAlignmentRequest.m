
classdef RBDLModelAlignmentRequest < ros.Message
    %RBDLModelAlignmentRequest MATLAB implementation of rbdl_server/RBDLModelAlignmentRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rbdl_server/RBDLModelAlignmentRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '07e770c1dc5b4e3e58279541dc6391a0' % The MD5 Checksum of the message definition
        PropertyList = { 'ModelName' 'Names' } % List of non-constant message properties
        ROSPropertyList = { 'model_name' 'names' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ModelName
        Names
    end
    methods
        function set.ModelName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'RBDLModelAlignmentRequest', 'ModelName');
            obj.ModelName = char(val);
        end
        function set.Names(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RBDLModelAlignmentRequest', 'Names');
            obj.Names = cell(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rbdl_server.RBDLModelAlignmentRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rbdl_server.RBDLModelAlignmentRequest(strObj);
        end
    end
end
