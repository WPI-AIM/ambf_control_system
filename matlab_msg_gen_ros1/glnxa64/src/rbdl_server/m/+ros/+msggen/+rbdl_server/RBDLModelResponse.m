
classdef RBDLModelResponse < ros.Message
    %RBDLModelResponse MATLAB implementation of rbdl_server/RBDLModelResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rbdl_server/RBDLModelResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '420aeb7337f0734e9777121e7c6c96a3' % The MD5 Checksum of the message definition
        PropertyList = { 'Good' } % List of non-constant message properties
        ROSPropertyList = { 'good' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Good
    end
    methods
        function set.Good(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RBDLModelResponse', 'Good');
            obj.Good = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rbdl_server.RBDLModelResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rbdl_server.RBDLModelResponse(strObj);
        end
    end
end
