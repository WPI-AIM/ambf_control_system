function [data, info] = rBDLJacobianRequest
%RBDLJacobian gives an empty data for rbdl_server/RBDLJacobianRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rbdl_server/RBDLJacobianRequest';
[data.ModelName, info.ModelName] = ros.internal.ros.messages.ros.char('string',0);
[data.BodyName, info.BodyName] = ros.internal.ros.messages.ros.char('string',0);
[data.Q, info.Q] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Point, info.Point] = ros.internal.ros.messages.geometry_msgs.point;
info.Point.MLdataType = 'struct';
info.MessageType = 'rbdl_server/RBDLJacobianRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'model_name';
info.MatPath{2} = 'body_name';
info.MatPath{3} = 'q';
info.MatPath{4} = 'point';
info.MatPath{5} = 'point.x';
info.MatPath{6} = 'point.y';
info.MatPath{7} = 'point.z';
