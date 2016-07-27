function [response_msg, returnCode] = callVREPService(robot, service, request)
%CALLVREPSERVICE emulates the execution of a service in V-REP
%   CALLVREPSERVICE emulates the execution of a service in V-REP
%
%   [response_msg, returnCode] = callVREPService(robot, service, request)
%
%   'returnCode' will depend on called internal function

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

    % Splitting the service name
    serv = strsplit(service, '/');
    
    % Switch the theservice name
    switch (char(serv(2)))
        case 'arm_1_joint'
            switch (char(serv(3)))
                case 'relax'
                    % Relaxing the arm_1_joint (arm joint)
                    returnCode   = robot.setWidowXJointRelaxed('arm');
                    response_msg = rosmessage('arbotix_msgs/RelaxResponse');
                    
                case 'enable'
                    % Enabling/disabling the arm_1_joint (arm joint)
                    returnCode   = robot.setWidowXJointEnabled('arm', request.Enable);
                    response_msg = rosmessage('arbotix_msgs/EnableResponse');
                    
                case 'set_speed'
                    % Setting max speed of the arm_1_joint (arm joint)
                    returnCode   = robot.setWidowXJointSpeed('arm', request.Speed);
                    response_msg = rosmessage('arbotix_msgs/SetSpeedResponse');
                    
                otherwise
                    error('%s %s %s', 'ERROR from "call":', v_serv, 'not supported');
            end
            
        case 'arm_2_joint'
            switch (char(serv(3)))
                case 'relax'
                    % Relaxing the arm_2_joint (shoulder joint)
                    returnCode   = robot.setWidowXJointRelaxed('shoulder');
                    response_msg = rosmessage('arbotix_msgs/RelaxResponse');
                    
                case 'enable'
                    % Enabling/disabling the arm_2_joint (shoulder joint)
                    returnCode   = robot.setWidowXJointEnabled('shoulder', request.Enable);
                    response_msg = rosmessage('arbotix_msgs/EnableResponse');
                    
                case 'set_speed'
                    % Setting max speed of the arm_2_joint (shouder joint)
                    returnCode   = robot.setWidowXJointSpeed('shoulder', request.Speed);
                    response_msg = rosmessage('arbotix_msgs/SetSpeedResponse');
                    
                otherwise
                    error('%s %s %s', 'ERROR from "call":', v_serv, 'not supported');
            end
            
        case 'arm_3_joint'
            switch (char(serv(3)))
                case  'relax'
                    % Relaxing the arm_3_joint (biceps joint)
                    returnCode   = robot.setWidowXJointRelaxed('biceps');
                    response_msg = rosmessage('arbotix_msgs/RelaxResponse');
                    
                case 'enable'
                    % Enabling/disabling the arm_3_joint (biceps joint)
                    returnCode   = robot.setWidowXJointEnabled('biceps', request.Enable);
                    response_msg = rosmessage('arbotix_msgs/EnableResponse');
                    
                case 'set_speed'
                    % Setting max speed of the arm_3_joint (biceps joint)
                    returnCode   = robot.setWidowXJointSpeed('biceps', request.Speed);
                    response_msg = rosmessage('arbotix_msgs/SetSpeedResponse');
                    
                otherwise
                    error('%s %s %s', 'ERROR from "call":', v_serv, 'not supported');
            end
            
        case 'arm_4_joint'
            switch (char(serv(3)))
                case 'relax'
                    % Relaxing the arm_4_joint (forearm joint)
                    returnCode   = robot.setWidowXJointRelaxed('forearm');
                    response_msg = rosmessage('arbotix_msgs/RelaxResponse');
                    
                case 'enable'
                    % Enabling/disabling the arm_4_joint (forearm joint)
                    returnCode   = robot.setWidowXJointEnabled('forearm', request.Enable);
                    response_msg = rosmessage('arbotix_msgs/EnableResponse');
                    
                case 'set_speed'
                    % Setting max speed of the arm_4_joint (forearm joint)
                    returnCode   = robot.setWidowXJointSpeed('forearm', request.Speed);
                    response_msg = rosmessage('arbotix_msgs/SetSpeedResponse');
                    
                otherwise
                    error('%s %s %s', 'ERROR from "call":', v_serv, 'not supported');
            end
        case 'arm_5_joint'
            switch (char(serv(3)))
                case 'relax'
                    % Relaxing the arm_5_joint (wrist joint)
                    returnCode   = robot.setWidowXJointRelaxed('wrist');
                    response_msg = rosmessage('arbotix_msgs/RelaxResponse');
                    
                case 'enable'
                    % Enabling/disabling the arm_5_joint (wrist joint)
                    returnCode   = robot.setWidowXJointEnabled('wrist', request.Enable);
                    response_msg = rosmessage('arbotix_msgs/EnableResponse');
                    
                case 'set_speed'
                    % Setting the max spped of the arm_5_joint (wrist joint)
                    returnCode   = robot.setWidowXJointSpeed('wrist', request.Speed);
                    response_msg = rosmessage('arbotix_msgs/SetSpeedResponse');
                    
                otherwise
                    error('%s %s %s', 'ERROR from "call":', v_serv, 'not supported');
            end
            
        case 'gripper_1_joint'
            switch (char(serv(3)))
                case 'relax'
                    % Relaxing the gripper_1_joint (gripper joint)
                    returnCode   = robot.setWidowXJointRelaxed('gripper');
                    response_msg = rosmessage('arbotix_msgs/RelaxResponse');
                    
                case 'enable'
                    % Enabling/disabling the gripper_1_joint (gripper joint)
                    returnCode   = robot.setWidowXJointEnabled('gripper', request.Enable);
                    response_msg = rosmessage('arbotix_msgs/EnableResponse');
                    
                case 'set_speed'
                    % Setting max speed of the gripper_1_joint (gripper joint)
                    returnCode   = robot.setWidowXJointSpeed('gripper', request.Speed);
                    response_msg = rosmessage('arbotix_msgs/SetSpeedResponse');
                    
                otherwise
                    error('%s %s %s', 'ERROR from "call":', v_serv, 'not supported');
            end

        otherwise
            error('%s %s %s', 'ERROR from "call":', v_serv, 'not supported');
    end
end