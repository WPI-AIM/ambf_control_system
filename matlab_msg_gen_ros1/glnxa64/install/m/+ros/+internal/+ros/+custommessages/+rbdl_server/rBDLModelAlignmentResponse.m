function [data, info] = rBDLModelAlignmentResponse
%RBDLModelAlignment gives an empty data for rbdl_server/RBDLModelAlignmentResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rbdl_server/RBDLModelAlignmentResponse';
[data.Names, info.Names] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Ids, info.Ids] = ros.internal.ros.messages.ros.default_type('int32',NaN);
info.MessageType = 'rbdl_server/RBDLModelAlignmentResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'names';
info.MatPath{2} = 'ids';
