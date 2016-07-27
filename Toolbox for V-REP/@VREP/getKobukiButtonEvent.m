function [v_msg, returnCode] = getKobukiButtonEvent(robot, timeout)
%GETKOBUKIBUTTONEVENT gets a message if a button event is ocurred
%within the time defined by 'timeout'.
%
%   [v_msg, returnCode] = getKobukiButtonEvent(robot, timeout)
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
%   uint8 Button0 = 0
%   uint8 Button1 = 1
%   uint8 Button2 = 2
%   uint8 RELEASED = 0
%   uint8 PRESSED  = 1
%   uint8 button
%   uint8 state

    % Initializing the event control variable
    event = 0;
 
    % Receiving sensors data from V-REP
    [~, statesData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
        'Turtlebot2_WidowX_kobuki_sensor_state', robot.vrep.simx_opmode_buffer);
    
    % Unpacking sensors data
    states = robot.vrep.simxUnpackFloats(statesData);

    % Getting the initial states of the buttons
    button_0_initial       = states(15);
    button_1_initial       = states(16);
    button_2_initial       = states(17);
    
    % Starting the time count
    tic;
    
    % While no event and timeout not elapsed
    while((toc < timeout) && (~event))
        % Getting the current sensor state data and unpacking it
        [~, statesData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
            'Turtlebot2_WidowX_kobuki_sensor_state', robot.vrep.simx_opmode_buffer);
        states = robot.vrep.simxUnpackFloats(statesData);
        
        % Getting the current states of the buttons
        button_0 = states(15);
        button_1 = states(16);
        button_2 = states(17);
        
        % If some button state changed, then an event occur
        if(button_0 ~= button_0_initial)
            button = 0;
            state = button_0;
            event = 1;   
        elseif (button_1 ~= button_1_initial)
            button = 1;
            state = button_1;
            event = 1;    
        elseif (button_2 ~= button_2_initial)
            button = 2;
            state = button_2;
            event = 1;    
        end
    end
    
    if(event)
        % Creating the message
        v_msg        = rosmessage('kobuki_msgs/ButtonEvent');
        v_msg.Button = uint8(button);
        v_msg.State  = uint8(state);
        returnCode   = 0;
    else        
        % Checking for errors
        returnCode = 1;     % No event
        if(result ~= robot.vrep.simx_return_ok)
            returnCode = -1;
        end 
    end   
end