function [data, info] = rBDLJacobianResponse
%RBDLJacobian gives an empty data for rbdl_server/RBDLJacobianResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rbdl_server/RBDLJacobianResponse';
[data.Jacobian, info.Jacobian] = ros.internal.ros.messages.std_msgs.float64MultiArray;
info.Jacobian.MLdataType = 'struct';
info.MessageType = 'rbdl_server/RBDLJacobianResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'jacobian';
info.MatPath{2} = 'jacobian.layout';
info.MatPath{3} = 'jacobian.layout.dim';
info.MatPath{4} = 'jacobian.layout.dim.label';
info.MatPath{5} = 'jacobian.layout.dim.size';
info.MatPath{6} = 'jacobian.layout.dim.stride';
info.MatPath{7} = 'jacobian.layout.data_offset';
info.MatPath{8} = 'jacobian.data';
