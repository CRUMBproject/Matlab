function [v_msg, returnCode] = getKobukiBumperEvent(robot, timeout)
%GETKOBUKIBUMPEREVENT gets a message if a bumper event is ocurred
%within the time defined by 'timeout'.
%
%   [v_msg, returnCode] = getKobukiBumperEvent(robot, timeout)
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

% Structure of the message
%   uint8 LEFT   = 0
%   uint8 CENTER = 1
%   uint8 RIGHT  = 2
%   uint8 RELEASED = 0
%   uint8 PRESSED  = 1
%   uint8 bumper
%   uint8 state

    % Initializing the event control variable
    event = 0;

    % Receiving sensors data from V-REP
    [~, statesData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
        'Turtlebot2_WidowX_kobuki_sensor_state', robot.vrep.simx_opmode_buffer);
  
    % Unpacking sensors data    
    states = robot.vrep.simxUnpackFloats(statesData);
    
    % Getting the initial states of the bumpers
    bumper_right_initial  = states(2);
    bumper_center_initial = states(3);
    bumper_left_initial   = states(4);
    
    % Starting the time count
    tic;
    
    % While no event and timeout not elapsed
    while((toc < timeout) && (~event))
        % Getting the current sensor state data and unpacking it
        [~, statesData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
            'Turtlebot2_WidowX_kobuki_sensor_state', robot.vrep.simx_opmode_buffer);  
        states = robot.vrep.simxUnpackFloats(statesData);
        
        % Getting the current states of the bumpers
        bumper_right  = states(2);
        bumper_center = states(3);
        bumper_left   = states(4);
        
        % If some bumper state changed, then an event occur
        if(bumper_right ~= bumper_right_initial)
            bumper = 2;
            state = bumper_right;
            event = 1;   
        elseif (bumper_center ~= bumper_center_initial)
            bumper = 1;
            state = bumper_center;
            event = 1;    
        elseif (bumper_left ~= bumper_left_initial)
            bumper = 0;
            state = bumper_left;
            event = 1;    
        end
    end
    
    if(event)
        % Creating the message
        v_msg        = rosmessage('kobuki_msgs/BumperEvent');
        v_msg.Bumper = uint8(bumper);
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