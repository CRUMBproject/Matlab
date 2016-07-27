function [v_msg, returnCode] = getKobukiControllerInfo(robot)
%GETKOBUKICONTROLLERINFO gets Kobuki's controller info. This info is the
%default info in the real robot, only for completeness. It is defined in
% V-REP model implementation. No actual meaning in the V-REP's robot model.
%
%   [v_msg, returnCode] = getKobukiControllerInfo(robot)
%
%   'returnCode' will be set to '0' if operation ends successfully or to 
%   '-1' if exits an error.

% -------------------------------------------------------------------------
% Ivan Fernandez-Vega
% B.Sc. Degree in Computer Engineering
% University of Malaga
% June, 2016
% -------------------------------------------------------------------------
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.
% -------------------------------------------------------------------------

% Structure of the message:
%   uint8 DEFAULT   =  0
%   uint8 USER_CONFIGURED =  1
%   uint8 type
%   float64 p_gain #should be positive
%   float64 i_gain #should be positive
%   float64 d_gain #should be positive

    % Getting controller info data from V-REP
    [result, controllerInfoData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
        'Turtlebot2_WidowX_kobuki_controller_info', robot.vrep.simx_opmode_buffer);
    
    % Unpacking the controller info data
    controllerInfo = robot.vrep.simxUnpackFloats(controllerInfoData);
    
    % Creating the message
    v_msg = rosmessage('kobuki_msgs/ControllerInfo');
    
    % Setting the message fields
    v_msg.Type  = uint8(controllerInfo(1));
    v_msg.PGain = controllerInfo(2);
    v_msg.IGain = controllerInfo(3);
    v_msg.DGain = controllerInfo(4);
    
    % Checking for errors
    returnCode = 0;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end 
end