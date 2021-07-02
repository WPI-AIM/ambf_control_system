function [data, info] = controllerListResponse
%ControllerList gives an empty data for controller_modules/ControllerListResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'controller_modules/ControllerListResponse';
[data.Controllers, info.Controllers] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'controller_modules/ControllerListResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'controllers';
