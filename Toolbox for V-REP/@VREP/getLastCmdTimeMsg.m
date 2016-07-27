function [v_msg, returnCode] = getLastCmdTimeMsg(robot)
%GETSIMULATIONTIME gets a message with the last command time.
%
% [v_msg, returnCode] = getLastCmdTimeMsg(robot)
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
%   float64 Data

    % Creating the message
    v_msg = rosmessage('std_msgs/Float64');

    % Getting the last command time
  	v_msg.Data = robot.vrep.simxGetLastCmdTime(robot.v_clientID);
    
    % Checking for erros
    returnCode = 0;
    if(v_msg.Data == 0) % It means that simulation is not running
        returnCode = -1;
    end  
end