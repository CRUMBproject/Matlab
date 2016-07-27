function [returnCode] = setKobukiVelocity(robot, linearX, angularZ)
%SETKOBUKIVELOCITY sets the target linear and angular speed for Kobuki.
%
%   [returnCode] = setKobukiVelocity(robot, linearX, angularZ)
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
    
    % Wheelbase = distance between the two wheels, in meters
    wheelbase = 0.23;
    
    % Wheel radius, in meters
    R = 0.035;
    
    % Each command has a sequence number (circular between 0 and 10241023)
    persistent lastCommand;
    if isempty(lastCommand) 
        lastCommand = 0;
    else
        lastCommand = mod(lastCommand + 1, 10241024); 
    end
    
    % Calculation of the speed for each motor
    leftMotorSpeed  = (linearX - (wheelbase / 2) * angularZ) / R;
    rightMotorSpeed = (linearX + (wheelbase / 2) * angularZ) / R; 
       
    % Packing the speed data
    wheelSpeedData = robot.vrep.simxPackFloats([rightMotorSpeed, ....
        leftMotorSpeed, lastCommand]);
    
    % Setting speed data to the signal in V-REP
    result = robot.vrep.simxSetStringSignal(robot.v_clientID, ...
        'Turtlebot2_WidowX_kobuki_wheels_speed', wheelSpeedData,...
        robot.vrep.simx_opmode_oneshot);   
       
    % Checking for errors
    returnCode = 0;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end 
end