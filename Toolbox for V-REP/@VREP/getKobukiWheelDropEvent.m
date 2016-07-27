function [v_msg, returnCode] = getKobukiWheelDropEvent(robot, timeout)
%GETKOBUKIWHEELDROPEVENT gets a message if a wheel drop event is ocurred
%within the time defined by 'timeout'.
%
%   [v_msg, returnCode] = getKobukiWheelDropEvent(robot, timeout)
%
%   'returnCode' will be set to '0' if operation ends successfully, to '1' 
%   if no events occurred or to '-1' if exits an error.

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

% Structure  of the message:
%   uint8 LEFT  = 0
%   uint8 RIGHT = 1
%   uint8 RAISED  = 0
%   uint8 DROPPED = 1
%   uint8 Wheel
%   uint8 State

    % Initialization of event control variable
    event = 0;

    % Getting the sensor state data
    [~, statesData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
            'Turtlebot2_WidowX_kobuki_sensor_state', robot.vrep.simx_opmode_buffer);
       
    % Unpacking the sensor state datta
    states = robot.vrep.simxUnpackFloats(statesData);
    
    % Saving the current wheel drops states
    wheel_drop_right_initial = states(5);
    wheel_drop_left_initial  = states(6);
    
    % Start the time count
    tic;    
    while((toc < timeout) && (~event))
        % Getting the current sensor state data and unpacking it
        [result, statesData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
            'Turtlebot2_WidowX_kobuki_sensor_state', robot.vrep.simx_opmode_oneshot_buffer);
        states = robot.vrep.simxUnpackFloats(statesData);
        
        % Getting the current wheel drops states
        wheel_drop_right = states(5);
        wheel_drop_left  = states(6);
        
        % If some wheel drop state changed, then an event occur
        if(wheel_drop_right ~= wheel_drop_right_initial)
            wheel_drop = 1;
            state = wheel_drop_right;
            event = 1;   
        elseif (wheel_drop_left ~= wheel_drop_left_initial)
            wheel_drop = 0;
            state = wheel_drop_left;
            event = 1;    
        end
    end
    
    % If event, we return a proper message
    if(event)
        v_msg = rosmessage('kobuki_msgs/WheelDropEvent');
        v_msg.Wheel = wheel_drop;
        v_msg.State = state;
        returnCode = 0;
    else
        % Checking for errors
        returnCode = 1;     % No event
        if(result ~= robot.vrep.simx_return_ok)
            returnCode = -1;
        end 
    end
end