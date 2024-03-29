
classdef RBDLModelAlignmentResponse < ros.Message
    %RBDLModelAlignmentResponse MATLAB implementation of rbdl_server/RBDLModelAlignmentResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rbdl_server/RBDLModelAlignmentResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'c48398e9db53c445e3bfe73fc7c106d1' % The MD5 Checksum of the message definition
        PropertyList = { 'Names' 'Ids' } % List of non-constant message properties
        ROSPropertyList = { 'names' 'ids' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Names
        Ids
    end
    methods
        function set.Names(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RBDLModelAlignmentResponse', 'Names');
            obj.Names = cell(val);
        end
        function set.Ids(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int32.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RBDLModelAlignmentResponse', 'Ids');
            obj.Ids = int32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rbdl_server.RBDLModelAlignmentResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rbdl_server.RBDLModelAlignmentResponse(strObj);
        end
    end
end
