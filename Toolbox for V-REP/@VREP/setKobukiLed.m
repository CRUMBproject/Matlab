function [returnCode] = setKobukiLed(robot, led, newState)
%SETKOBUKILED sets the state of a defined Kobuki's LED.
%
%   [returnCode] = setKobukiLed(robot, led, newState)
%
%   'returnCode' will be set to '0' if operation ends successfully, to '1'
%   if led not recognized or to '-1' if exits other error.

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
    
    % Initializating error and result variables
    error  = 0;
    result = -1; 

    % Searching for the desired LED
    if(led == 1)
        result = robot.vrep.simxSetIntegerSignal(robot.v_clientID, ...
            'Turtlebot2_WidowX_kobuki_led_1', ...
            newState,robot.vrep.simx_opmode_oneshot);  
        
    elseif (led == 2)
        result = robot.vrep.simxSetIntegerSignal(robot.v_clientID, ...
            'Turtlebot2_WidowX_kobuki_led_2', ...
           newState,robot.vrep.simx_opmode_oneshot);
       
    else
        % Not a valid LED number
        error = 1;
        
    end
    
    % Checking for errors
    returnCode = error;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end
end