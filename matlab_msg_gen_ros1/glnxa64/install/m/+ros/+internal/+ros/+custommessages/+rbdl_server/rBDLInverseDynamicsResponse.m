function [data, info] = rBDLInverseDynamicsResponse
%RBDLInverseDynamics gives an empty data for rbdl_server/RBDLInverseDynamicsResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rbdl_server/RBDLInverseDynamicsResponse';
[data.Tau, info.Tau] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'rbdl_server/RBDLInverseDynamicsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'tau';
