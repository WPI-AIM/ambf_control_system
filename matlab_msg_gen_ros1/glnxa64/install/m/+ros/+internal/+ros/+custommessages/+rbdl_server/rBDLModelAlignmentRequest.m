function [data, info] = rBDLModelAlignmentRequest
%RBDLModelAlignment gives an empty data for rbdl_server/RBDLModelAlignmentRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rbdl_server/RBDLModelAlignmentRequest';
[data.ModelName, info.ModelName] = ros.internal.ros.messages.ros.char('string',0);
[data.Names, info.Names] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rbdl_server/RBDLModelAlignmentRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'model_name';
info.MatPath{2} = 'names';
