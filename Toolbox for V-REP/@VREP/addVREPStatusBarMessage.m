function [returnCode] = addVREPStatusBarMessage(robot, message)
%ADDVREPSTATUSBARMESSAGE Adds a message in V-REP's status bar
%   [returnCode] = ADDVREPSTATUSBARMESSAGE(robot, message) adds the message
%   contained in the string 'message' using the object 'robot'.
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
    
    % Adding the message
    result = robot.vrep.simxAddStatusbarMessage(robot.v_clientID, ...   
        message, robot.vrep.simx_opmode_oneshot);
    
    % Checking for errors
    returnCode = 0;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end    
end