% /////////////////////////////////////////////////////////////////////////
% #########################################################################
%
%                    VREP and ROS COMMUNICATION CLASS   
%
% #########################################################################
% \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

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

classdef VREP<handle
    %VREP is a class that combines V-REP simulation and ROS communication
    %with the same program
    
    properties 
        % Miscellaneous
        operating_mode;     % 0->V-REP, 1->REAL ROBOT, 2->MIXED
        noise_enabled;      % Noise generation in MATLAB
      
        % V-REP properties
        v_ip_addr;
        v_port;
        vrep;
        v_clientID;
        v_topics;
        v_topics_types;
        
        v_services; 
        v_handles;
        v_console;
        
        hokuyo_last_measure;
                        
        % ROS properties
        r_ip_addr;
        r_port;
        
        % Other
        header_seq_number;
    end
    
    methods
        % Constructor
        function robot = VREP(arg0, arg1)
            if  nargin > 0
                if (strcmp(arg0, 'vrep'))
                    robot.operating_mode = 0;
                elseif (strcmp(arg0, 'real'))
                    robot.operating_mode = 1;
                elseif (strcmp(arg0, 'mixed'))
                    robot.operating_mode = 2;
                else
                   error(strcat(arg0,...
                       ' is an invalid operation mode. Please choose vrep, real or mixed.'))  
                end
                if nargin == 1
                    robot.noise_enabled = 1;
                else
                    if strcmp(arg1, 'noise')
                        robot.noise_enabled = 1;
                    elseif strcmp(arg1, 'noiseless')
                        robot.noise_enabled = 0;
                    end                    
                end
                
            else
                robot.operating_mode = 0;
            end
        end
        % Pseudo-ROS functions
        [returnCode]            = rosinit(robot, v_ip_addr, v_port, r_ip_addr, r_port);        
        [out_1, returnCode]     = rostopic(robot, arg0, arg1)
        [returnCode]            = rosservice(robot, arg0);
        [returnCode]            = rosshutdown(robot);       
        [publisher, returnCode] = rospublisher(robot, topiname)
        [out_1, out_2, out_3]   = rossubscriber(robot, in_1);
        [client, returnCode]    = rossvcclient(robot, servicename);
        [msg, returnCode]       = rosmessage(robot, entity);
        [returnCode]            = send(robot, in_1, in_2, in_3, in_4)
        [out_1, out_2, out_3]   = receive(robot, in_1, in_2, in_3);
        [out_1, out_2, out_3]   = call(robot, in_1, in_2);
        
        % Other funtions
        [pingTime, returnCode]      = getPingTime(robot);
        [returnCode]                = pause(robot, pauseTime);
        [enabled]                   = isVREPEnabled(robot)
        [enabled]                   = isROSEnabled(robot)
        
    end        
    
    % Private functions
    methods (Access = private)
        
        % Miscellaneous
        [header, returnCode]       = getMessageHeader(robot);
        [returnCode]               = acquireVREPHandlers(robot);
        [message, returnCode]      = getVREPMessage(robot, publish);
        [msg, returnCode]          = receiveFromVREP(robot, subscrib, timeout);
        [returnCode]               = sendToVREP(robot, publish, msg);
        [returnCode]               = setVREPTopics(robot);
        [returnCode]               = setVREPServices(robot);
        [response_msg, returnCode] = callVREPService(robot, service, request)
        [msg_type]                 = rostopicTypeVREP(robot, topic);      

        
        % KOBUKI
            % Get
            [v_msg, returnCode] = getKobukiOdometryMsg(robot);
            [v_msg, returnCode] = getKobukiSensorState(robot);
            [v_msg, returnCode] = getJointStatesMsg(robot);
            [v_msg, returnCode] = getKobukiImuMsg(robot);
            [v_msg, returnCode] = getKobukiImuRawMsg(robot);
            [v_msg, returnCode] = getKobukiControllerInfo(robot);
            [v_msg, returnCode] = getKobukiVersionInfo(robot);
            [v_msg, returnCode] = getKobukiBumperEvent(robot, timeout);
            [v_msg, returnCode] = getKobukiButtonEvent(robot, timeout);
            [v_msg, returnCode] = getKobukiCliffEvent(robot, timeout);
            [v_msg, returnCode] = getKobukiWheelDropEvent(robot, timeout);
                       
            % Set
            [returnCode]        = setKobukiVelocity(robot, linearX, angularZ);  
            [returnCode]        = setKobukiLed(robot, led, newState);
            [returnCode]        = setKobukiMotorsEnabled(robot, state);
            
            % Reset
            [returnCode]        = resetKobukiOdometry(robot);
            
        % KINECT
            [v_msg, returnCode] = getKinectImageMsg(robot, depth, color, compressed);  
        
        % HOKUYO    
            [v_msg, returnCode] = getHokuyoDataMsg(robot);
        
        % WIDOWX
            [returnCode]           = setWidowXJointPosition(robot, joint, newState);
            [returnCode]           = setWidowXJointRelaxed(robot, joint);
            [returnCode]           = setWidowXJointEnabled(robot, joint, enabled);
            [returnCode]           = setWidowXJointSpeed(robot, joint, speed);
            [position, returnCode] = getWidowXJointPosition(robot, joint);
        
        % Simulation
            [v_msg, returnCode]    = getKobukiSimulationPoseMsg(robot);
            [simTime, returnCode]  = getSimulationTime(robot);
            [v_msg, returnCode]    = getLastCmdTimeMsg(robot);
            [v_msg, returnCode]    = getSimulationTimeMsg(robot);
            [returnCode]           = setPauseCommunicationState(robot, state);
            [returnCode]           = addVREPStatusBarMessage(robot, message);
            [returnCode]           = manageAuxiliaryVREPConsole(robot, command, message);
    end    
end