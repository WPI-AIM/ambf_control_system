function [data, info] = rBDLBodyNamesResponse
%RBDLBodyNames gives an empty data for rbdl_server/RBDLBodyNamesResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rbdl_server/RBDLBodyNamesResponse';
[data.Names, info.Names] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'rbdl_server/RBDLBodyNamesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'names';
