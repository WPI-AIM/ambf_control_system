function [data, info] = rBDLInverseKinimaticsResponse
%RBDLInverseKinimatics gives an empty data for rbdl_server/RBDLInverseKinimaticsResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rbdl_server/RBDLInverseKinimaticsResponse';
[data.QRes, info.QRes] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Worked, info.Worked] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'rbdl_server/RBDLInverseKinimaticsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'q_res';
info.MatPath{2} = 'worked';
