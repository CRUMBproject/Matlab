function [v_msg, returnCode] = getJointStatesMsg(robot)
%GETJOINTSTATES returns the joint states message according to the current
%state of robot's joints
%
%   [v_msg, returnCode] = getJointStatesMsg(robot)
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

    % Checking if this is the first call
    persistent lastCall;
    if isempty(lastCall) 
        lastCall = 0;
    end
    
    % Receiving the joint_states data packet from V-REP
    [result, statesData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
        'Turtlebot2_WidowX_joint_states', robot.vrep.simx_opmode_buffer);
    
    % Unpacking the data    
    states = robot.vrep.simxUnpackFloats(statesData);
    
    % Creating the message
    v_msg = rosmessage('sensor_msgs/JointState');
    
    % Getting a message header
    v_msg.Header = robot.getMessageHeader();
    
    % Kobuki's joints will be published twice more than WidowX's joints
    if(lastCall <= 1)
        % Kobuki joints
        v_msg.Name = {'wheel_left_joint', 'wheel_right_joint'};
        v_msg.Position = double([states(1), states(4)]);
        v_msg.Velocity = double([states(2), states(5)]);
        v_msg.Effort   = double([states(3), states(6)]);      
        
        lastCall = lastCall + 1;
    else
        % WidowX joints
        v_msg.Name = { 'arm_1_joint', 'arm_2_joint', 'arm_3_joint',...
            'gripper_1_joint', 'arm_4_joint', 'arm_5_joint'};
        v_msg.Position = double([states(7), states(10), states(13), ...
             states(22), states(16), states(19)]);
        v_msg.Velocity = double([states(8), states(11), states(14), ...
            states(23), states(17), states(20)]);
        v_msg.Effort   = [];  

        lastCall = 0;
    end  
    
    % Checking for errors
    returnCode = 0;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end 
end