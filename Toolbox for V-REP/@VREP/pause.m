function [returnCode] = pause(robot, pauseTime)
%PAUSE will pause the execution of the current program according to the
%selected operating mode
%
%   [returnCode] = pause(robot, pauseTime)
%
%   If selected operating mode includes V-REP, this function will pause the
%   program exection with simulation time (this not have to be the same to
%   'real time'.
%   Otherwise, 'pause' function from MATLAB will be called.
%
%   'returnCode' will be set always to 0.

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

    if robot.operating_mode == 0 || robot.operating_mode == 2 % V-REP and MIXED
        % Getting the start time
        startTime = robot.getSimulationTime();
    
        % Waiting that pauseTime elapses in simulation
        while ((robot.getSimulationTime() - startTime) < pauseTime), end;
        
    else  % ROS   
        pause(pauseTime);
    end       
    returnCode = 0;
end