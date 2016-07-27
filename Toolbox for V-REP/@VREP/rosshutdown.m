function [returnCode] = rosshutdown(robot)
%ROSSHUTDOWN Shuts down the connection between MATLAB and V-REP and/or between
%MATLAB and ROS (depending of selected operating mode).
%
%   [returnCode] = rosshutdown(robot)
%
%   'returnCode' will allways be set to '0' (no errors)

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

    % REAL ROBOT            
    if(robot.operating_mode == 1 || robot.operating_mode == 2)
        rosshutdown;
    end
            
    % V-REP            
    if(robot.operating_mode == 0 || robot.operating_mode == 2)
        % User's notification
        robot.vrep.simxAddStatusbarMessage(robot.v_clientID, ...
            'Disconnected from Matlab.', robot.vrep.simx_opmode_oneshot);
       	if (robot.v_clientID > -1)
            % Before closing the connection to V-REP, make sure that the 
            % last command sent out had time to arrive. It can be
            % guaranteed with (for example):
            robot.vrep.simxGetPingTime(robot.v_clientID);
            robot.vrep.simxFinish(robot.v_clientID);
        end
        robot.vrep.delete();
    end
    
    % No errors
    returnCode = 0;
end