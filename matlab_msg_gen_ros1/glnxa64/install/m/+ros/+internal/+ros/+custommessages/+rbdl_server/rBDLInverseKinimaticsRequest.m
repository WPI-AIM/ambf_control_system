function [data, info] = rBDLInverseKinimaticsRequest
%RBDLInverseKinimatics gives an empty data for rbdl_server/RBDLInverseKinimaticsRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rbdl_server/RBDLInverseKinimaticsRequest';
[data.ModelName, info.ModelName] = ros.internal.ros.messages.ros.char('string',0);
[data.BodyName, info.BodyName] = ros.internal.ros.messages.ros.char('string',0);
[data.Target, info.Target] = ros.internal.ros.messages.geometry_msgs.point;
info.Target.MLdataType = 'struct';
info.MessageType = 'rbdl_server/RBDLInverseKinimaticsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'model_name';
info.MatPath{2} = 'body_name';
info.MatPath{3} = 'target';
info.MatPath{4} = 'target.x';
info.MatPath{5} = 'target.y';
info.MatPath{6} = 'target.z';
