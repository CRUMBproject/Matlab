function [v_msg, returnCode] = getSimulationTimeMsg(robot)
%GETSIMULATIONTIMEMSG gets a message with the current simulation time in V-REP
%
%   [v_msg, returnCode] = getSimulationTimeMsg(robot))
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
%   float64 data

    % Getting the message
    v_msg = rosmessage('std_msgs/Float64');
    
    % Getting the simulation time and setting to the message
    [result, v_msg.Data] = robot.vrep.simxGetFloatSignal(robot.v_clientID, ...
        'Turtlebot2_WidowX_simulation_time', robot.vrep.simx_opmode_buffer);  
    
    % Checking for errors
    returnCode = 0;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end 
end