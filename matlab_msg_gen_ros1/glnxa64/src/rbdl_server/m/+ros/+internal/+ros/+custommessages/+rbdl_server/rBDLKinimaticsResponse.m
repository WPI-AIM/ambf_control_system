function [data, info] = rBDLKinimaticsResponse
%RBDLKinimatics gives an empty data for rbdl_server/RBDLKinimaticsResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'rbdl_server/RBDLKinimaticsResponse';
[data.Names, info.Names] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Points, info.Points] = ros.internal.ros.messages.geometry_msgs.point;
info.Points.MLdataType = 'struct';
info.Points.MaxLen = NaN;
info.Points.MinLen = 0;
data.Points = data.Points([],1);
info.MessageType = 'rbdl_server/RBDLKinimaticsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'names';
info.MatPath{2} = 'points';
info.MatPath{3} = 'points.x';
info.MatPath{4} = 'points.y';
info.MatPath{5} = 'points.z';
