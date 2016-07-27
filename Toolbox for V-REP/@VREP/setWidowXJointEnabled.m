function [returnCode] = setWidowXJointEnabled(robot, joint, enabled)
%SETWIDOWXJOINTRELAXED sets the desired enabled state of a defined joint of
%WidowX.
%
%   [returnCode] = setWidowXJointEnabled(robot, joint, enabled)
%
%   'returnCode' will be set to '0' if operation ends successfully, to '1'
%   if joint not recognized or to '-1' if exits other error.

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
    
    % Searching for the desired joint
    switch(joint)
        case 'arm'
            result = robot.vrep.simxSetIntegerSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_widowx_shoulder_joint_conf', enabled, ...
                robot.vrep.simx_opmode_oneshot);   
        
        case 'shoulder'
            result = robot.vrep.simxSetIntegerSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_widowx_shoulder_joint_conf', enabled, ...
                robot.vrep.simx_opmode_oneshot);
   
        case 'biceps'
            result = robot.vrep.simxSetIntegerSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_widowx_biceps_joint_conf', enabled, ...
                robot.vrep.simx_opmode_oneshot);  
        
        case 'forearm'
            result = robot.vrep.simxSetIntegerSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_widowx_forearm_joint_conf', enabled, ...
                robot.vrep.simx_opmode_oneshot);  

        case 'wrist'
            result = robot.vrep.simxSetIntegerSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_widowx_wrist_joint_conf', enabled, ...
                robot.vrep.simx_opmode_oneshot);
               
        case 'gripper'
            result = robot.vrep.simxSetIntegerSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_widowx_gripper_joint_conf', enabled, ...
                robot.vrep.simx_opmode_oneshot);
            
        otherwise
            error = 1;
    end
    
    % Checking for errors
    returnCode = error;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end  
end