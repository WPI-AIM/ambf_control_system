function [data, info] = rBDLModelResponse
%RBDLModel gives an empty data for rbdl_server/RBDLModelResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rbdl_server/RBDLModelResponse';
[data.Good, info.Good] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'rbdl_server/RBDLModelResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'good';
