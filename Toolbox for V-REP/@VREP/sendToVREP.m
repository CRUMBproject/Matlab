function [returnCode] = sendToVREP(robot, publish, msg)
%SENDTOVREP sends a message to VREP, according to specified publisher.
%
%   [returnCode] = sendToVREP(robot, publish, msg)
%
%   'returnCode' will depend on selected publisher. In case of error (topic
%   not implemented) program will end

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
    
    % Initialization of 'returnCode'
    returnCode = -2;
    
    % Splitting the publisher
    publisher = strsplit(publish, '/');
    
    % Searching for the topic
    switch(char(publisher(2)))
        case 'mobile_base' 
            switch(char(publisher(3)))
                case 'commands'
                    switch(char(publisher(4)))
                        case 'velocity'
                            returnCode = setKobukiVelocity(robot, ...
                                msg.Linear.X, msg.Angular.Z);
                            
                        case 'led1'
                            returnCode = setKobukiLed(robot, 1, msg.Value);
                            
                        case 'led2'
                            returnCode = setKobukiLed(robot, 2, msg.Value);
                            
                        case 'motor_power'
                            returnCode = setKobukiMotorsEnabled(robot, msg.State); 
                            
                        case 'reset_odometry'
                            returnCode = resetKobukiOdometry(robot);
                    end
            end
        case 'arm_1_joint'
            switch(char(publisher(3)))
                case 'command'
                    returnCode = setWidowXJointPosition(robot, 'arm', msg.Data);
            end
            
        case 'arm_2_joint'
            switch(char(publisher(3)))
                case 'command'
                    returnCode = setWidowXJointPosition(robot, 'shoulder', msg.Data);
            end  
            
        case 'arm_3_joint'
            switch(char(publisher(3)))
                case 'command'
                    returnCode = setWidowXJointPosition(robot, 'biceps', msg.Data);
            end
            
        case 'arm_4_joint'
            switch(char(publisher(3)))
                case 'command'
                    returnCode = setWidowXJointPosition(robot, 'forearm', msg.Data);
            end
            
        case 'arm_5_joint'
            switch(char(publisher(3)))
                case 'command'
                    returnCode = setWidowXJointPosition(robot, 'wrist', msg.Data);
            end
            
        case 'gripper_1_joint'
            switch(char(publisher(3)))
                case 'command'
                    returnCode = setWidowXJointPosition(robot, 'gripper', msg.Data);
            end
            
        case 'vrep'
            switch(char(publisher(3)))
                case 'status_bar_message'
                    returnCode = addVREPStatusBarMessage(robot, msg.Data);
                    
                case 'aux_console'
                    switch(char(publisher(4)))
                        case 'print'
                            returnCode = robot.manageAuxiliaryVREPConsole...
                                ('print', msg.Data);                            
                        otherwise
                            % clear, create, delete, show, hide
                            returnCode = robot.manageAuxiliaryVREPConsole...
                                (char(publisher(4)));
                    end                    
                case 'pause_communication'                    
                    returnCode = robot.setPauseCommunicationState(msg.Data);
            end
    end
    % Checking for errors
    if(returnCode == -2)
        error('SEND: not implemented topic');
    end
end
