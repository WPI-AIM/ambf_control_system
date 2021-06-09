function [data, info] = jointControlResponse
%JointControl gives an empty data for controller_modules/JointControlResponse
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'controller_modules/JointControlResponse';
[data.ControlOutput, info.ControlOutput] = ros.internal.ros.messages.trajectory_msgs.jointTrajectoryPoint;
info.ControlOutput.MLdataType = 'struct';
info.MessageType = 'controller_modules/JointControlResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'control_output';
info.MatPath{2} = 'control_output.positions';
info.MatPath{3} = 'control_output.velocities';
info.MatPath{4} = 'control_output.accelerations';
info.MatPath{5} = 'control_output.effort';
info.MatPath{6} = 'control_output.time_from_start';
info.MatPath{7} = 'control_output.time_from_start.sec';
info.MatPath{8} = 'control_output.time_from_start.nsec';
