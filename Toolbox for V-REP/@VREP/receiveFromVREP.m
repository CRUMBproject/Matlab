function [v_msg, returnCode] = receiveFromVREP(robot, subscrib, timeout)
%RECEIVEFROMVREP receives a message from V-REP depending of the subscriber.
%
%   [v_msg, returnCode] = receiveFromVREP(robot, subscrib, timeout)
%
%   'returnCode' will be set to 'returnCode' of called function. In case of
%   error (topic not recognized), program will end.

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

    % Splitting the subscriber
    subscriber = strsplit(subscrib, '/');
    
    % Searching the subscriber and calling the proper function
    switch (char(subscriber(2)))
        case 'odom'
            [v_msg, returnCode] = robot.getKobukiOdometryMsg();
            
        case 'joint_states'
            [v_msg, returnCode] = robot.getJointStatesMsg();
                
        case 'scan'
            [v_msg, returnCode] = robot.getHokuyoDataMsg();
   
        case 'mobile_base'
            switch (char(subscriber(3)))
                case 'sensors'
                    switch (char(subscriber(4)))
                        case 'core'
                            [v_msg, returnCode] = robot.getKobukiSensorState();
                                
                        case 'imu_data'
                            [v_msg, returnCode] = robot.getKobukiImuMsg();
                                
                        case 'imu_data_raw'
                            [v_msg, returnCode] = robot.getKobukiImuRawMsg();                            
                    end
                case 'events'
                    switch (char(subscriber(4)))
                        case 'bumper'
                            [v_msg, returnCode] = robot.getKobukiBumperEvent(timeout);
                            
                        case 'button'
                            [v_msg, returnCode] = robot.getKobukiButtonEvent(timeout);
                            
                        case 'cliff'
                            [v_msg, returnCode] = robot.getKobukiCliffEvent(timeout);
                    
                        case 'wheel_drop'
                            [v_msg, returnCode] = robot.getKobukiWheelDropEvent(timeout);
                    end
                case 'controller_info'
                    [v_msg, returnCode] = robot.getKobukiControllerInfo();
                
                case 'version_info'
                    [v_msg, returnCode] = robot.getKobukiVersionInfo();
            end
        case 'camera'
            switch(char(subscriber(3)))
                case 'rgb'
                    switch(char(subscriber(4)))
                        case 'image_color'
                            if(size(subscriber,2) == 4)
                                [v_msg, returnCode] = robot.getKinectImageMsg(0, 0, 0);
                                
                            elseif(strcmp(subscriber(5), 'compressed'))
                                [v_msg, returnCode] = robot.getKinectImageMsg(0, 0, 1);
                                
                            end
                            
                        case 'image_mono'
                            if(size(subscriber, 2) == 4)
                                [v_msg, returnCode] = robot.getKinectImageMsg(0, 1, 0);
                                
                            elseif(strcmp(subscriber(5), 'compressed'))
                                [v_msg, returnCode] = robot.getKinectImageMsg(0, 1, 1);
                                
                            end
                    end
                    
                case 'depth'
                    switch(char(subscriber(4)))
                        case 'image_raw'
                            if(size(subscriber,2) == 4)
                                [v_msg, returnCode] = robot.getKinectImageMsg(1, 1, 0);
                                
                            elseif(strcmp(subscriber(5), 'compressed'))  
                                [v_msg, returnCode] = robot.getKinectImageMsg(1, 1, 1);
                                
                            end
                    end
            end
            
        case 'simulation'
            switch(char(subscriber(3)))
                case 'pose'
                    [v_msg, returnCode] = robot.getKobukiSimulationPoseMsg();
                    
                case 'sim_time'
                    [v_msg, returnCode] = robot.getSimulationTimeMsg();
                    
            end
            
        case 'vrep'            
            switch(char(subscriber(3)))
                case 'last_cmd_time'
                    [v_msg, returnCode] = robot.getLastCmdTimeMsg();
                    
            end
            
        otherwise
            error('RECEIVE: topic not implemented.');
            
    end
end
