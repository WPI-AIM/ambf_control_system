
classdef RBDLForwardDynamicsRequest < ros.Message
    %RBDLForwardDynamicsRequest MATLAB implementation of rbdl_server/RBDLForwardDynamicsRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rbdl_server/RBDLForwardDynamicsRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'f127ad30f28d40445e1e732a1f350582' % The MD5 Checksum of the message definition
        PropertyList = { 'ModelName' 'Q' 'Qd' 'Tau' } % List of non-constant message properties
        ROSPropertyList = { 'model_name' 'q' 'qd' 'tau' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ModelName
        Q
        Qd
        Tau
    end
    methods
        function set.ModelName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'RBDLForwardDynamicsRequest', 'ModelName');
            obj.ModelName = char(val);
        end
        function set.Q(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RBDLForwardDynamicsRequest', 'Q');
            obj.Q = double(val);
        end
        function set.Qd(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RBDLForwardDynamicsRequest', 'Qd');
            obj.Qd = double(val);
        end
        function set.Tau(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RBDLForwardDynamicsRequest', 'Tau');
            obj.Tau = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rbdl_server.RBDLForwardDynamicsRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rbdl_server.RBDLForwardDynamicsRequest(strObj);
        end
    end
end
