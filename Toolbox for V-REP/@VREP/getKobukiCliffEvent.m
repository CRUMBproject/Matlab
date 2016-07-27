function [v_msg, returnCode] = getKobukiCliffEvent(robot, timeout)
%GETKOBUKICLIFFEVENT gets a message if a cliff event is ocurred
%within the time defined by 'timeout'.
%
%   [v_msg, returnCode] = getKobukiCliffEvent(robot, timeout)
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

% Structure of the message:
%   uint8 LEFT   = 0
%   uint8 CENTER = 1
%   uint8 RIGHT  = 2
%   uint8 FLOOR = 0
%   uint8 CLIFF = 1
%   uint8 Sensor
%   uint8 State
%   uint16 Bottom

    % Initializing the event control variable
    event = 0;

    % Receiving sensors data from V-REP
    [~, statesData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
        'Turtlebot2_WidowX_kobuki_sensor_state', robot.vrep.simx_opmode_buffer);
   
    % Unpacking sensors data
    states = robot.vrep.simxUnpackFloats(statesData);    
    
    % Getting the initial states of the cliffs
    cliff_right_initial  = states(7);
    cliff_center_initial = states(8);
    cliff_left_initial   = states(9);

    % Starting the time count
    tic;
    
    % While no event and timeout not elapsed
    while((toc < timeout) && (~event))
        % Getting the current sensor state data and unpacking it
        [result, statesData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
            'Turtlebot2_WidowX_kobuki_sensor_state', robot.vrep.simx_opmode_buffer);
        states = robot.vrep.simxUnpackFloats(statesData);
        
        % Getting the current states of the cliff sensors
        cliff_right  = states(7);
        cliff_center = states(8);
        cliff_left   = states(9);
             
        % If some cliff state changed, then an event occur
        if(cliff_right ~= cliff_right_initial)
            cliff = 2;
            state = cliff_right;
            bottom = states(10);
            event = 1;   
        elseif (cliff_center ~= cliff_center_initial)
            cliff = 1;
            state = cliff_center;
            bottom = states(11);
            event = 1;    
        elseif (cliff_left ~= cliff_left_initial)
            cliff = 0;
            state = cliff_left;
            bottom = states(12);
            event = 1;    
        end
    end
    
    if(event)
        % Creating the message
        v_msg        = rosmessage('kobuki_msgs/CliffEvent');
        v_msg.Sensor = uint8(cliff);
        v_msg.State  = uint8(state);
        v_msg.Bottom = uint16(bottom);
        returnCode   = 0;
    else        
        % Checking for errors
        returnCode = 1;     % No event
        if(result ~= robot.vrep.simx_return_ok)
            returnCode = -1;
        end 
    end
end