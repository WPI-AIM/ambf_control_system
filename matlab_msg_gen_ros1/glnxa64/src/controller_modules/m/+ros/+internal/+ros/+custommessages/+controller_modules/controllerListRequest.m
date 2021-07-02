function [data, info] = controllerListRequest
%ControllerList gives an empty data for controller_modules/ControllerListRequest
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'controller_modules/ControllerListRequest';
info.MessageType = 'controller_modules/ControllerListRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
