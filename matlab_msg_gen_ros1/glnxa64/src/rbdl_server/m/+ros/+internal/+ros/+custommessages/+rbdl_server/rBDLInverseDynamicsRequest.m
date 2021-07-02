function [data, info] = rBDLInverseDynamicsRequest
%RBDLInverseDynamics gives an empty data for rbdl_server/RBDLInverseDynamicsRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rbdl_server/RBDLInverseDynamicsRequest';
[data.ModelName, info.ModelName] = ros.internal.ros.messages.ros.char('string',0);
[data.Q, info.Q] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Qd, info.Qd] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Qdd, info.Qdd] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'rbdl_server/RBDLInverseDynamicsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'model_name';
info.MatPath{2} = 'q';
info.MatPath{3} = 'qd';
info.MatPath{4} = 'qdd';
