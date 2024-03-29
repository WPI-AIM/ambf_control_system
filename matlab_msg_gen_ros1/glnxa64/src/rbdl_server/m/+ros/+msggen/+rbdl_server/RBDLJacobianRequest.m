
classdef RBDLJacobianRequest < ros.Message
    %RBDLJacobianRequest MATLAB implementation of rbdl_server/RBDLJacobianRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rbdl_server/RBDLJacobianRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '0cdca2dc8b6b02adddd75186e68d6a82' % The MD5 Checksum of the message definition
        PropertyList = { 'Point' 'ModelName' 'BodyName' 'Q' } % List of non-constant message properties
        ROSPropertyList = { 'point' 'model_name' 'body_name' 'q' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.Point' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Point
        ModelName
        BodyName
        Q
    end
    methods
        function set.Point(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point'};
            validateattributes(val, validClasses, validAttributes, 'RBDLJacobianRequest', 'Point')
            obj.Point = val;
        end
        function set.ModelName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'RBDLJacobianRequest', 'ModelName');
            obj.ModelName = char(val);
        end
        function set.BodyName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'RBDLJacobianRequest', 'BodyName');
            obj.BodyName = char(val);
        end
        function set.Q(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RBDLJacobianRequest', 'Q');
            obj.Q = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rbdl_server.RBDLJacobianRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rbdl_server.RBDLJacobianRequest(strObj);
        end
    end
end
