function [v_msg, returnCode] = getKobukiSimulationPoseMsg(robot)
%GETKOBUKISIMULATIONPOSE gets the 'real' Kobuki pose according to
%simulator's calculation.
%
%   [v_msg, returnCode] = getKobukiSimulationPoseMsg(robot)
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
%   geometry_msgs/Point position
%       float64 x
%       float64 y
%       float64 z
%   geometry_msgs/Quaternion orientation
%       float64 x
%       float64 y
%       float64 z
%       float64 w
    
    % Getting the Kobuki's position
    [result1, position] = robot.vrep.simxGetObjectPosition(robot.v_clientID,...
        robot.v_handles(9, 1), -1, robot.vrep.simx_opmode_buffer);
    
    % Getting the Kobuki's orientation in Euler angles
    [result2, orientation] = robot.vrep.simxGetObjectOrientation(robot.v_clientID,...
        robot.v_handles(9, 1), -1, robot.vrep.simx_opmode_buffer);
    
    % Calculating the quaternion
    quaternion = eul2quat(orientation);
        
    % Creating the message
    v_msg = rosmessage('geometry_msgs/Pose');
    
    % Setting the position fields of the message
    v_msg.Position.X = position(1);
    v_msg.Position.Y = position(2);
    v_msg.Position.Z = position(3);

    % Setting the quaternion fields of the message
    v_msg.Orientation.X = quaternion(1);
    v_msg.Orientation.Y = quaternion(2);
    v_msg.Orientation.Z = quaternion(3);
    v_msg.Orientation.W = quaternion(4);    
    
    % Checking for errors
    returnCode = 0;
    if(result1 ~= robot.vrep.simx_return_ok || result2 ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end 
end