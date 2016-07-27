function [message, returnCode] = getVREPMessage(robot, publish)
%GETVREPMESSAGE gets the empty message for the defined V-REP's publisher.
%
%   [message, returnCode] = getVREPMessage(robot, publish)
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
    
    % Initializing the return value
    returnCode = 0;

    % Splitting the publisher
	publisher = strsplit(publish, '/');
    
    % Searching for the publisher
    switch (char(publisher(2)))        
        case 'mobile_base'
            if(strcmp(publisher(3), 'commands'))
                if(strcmp(publisher(4), 'velocity'))
                    message = rosmessage('geometry_msgs/Twist');
                    
                elseif (strcmp(publisher(4), 'led1') ||...
                        strcmp(publisher(4), 'led2'))
                    message = rosmessage('kobuki_msgs/Led');
                    
                elseif (strcmp(publisher(4), 'motor_power'))
                    message = rosmessage('kobuki_msgs/MotorPower');
                    
                elseif (strcmp(publisher(4), 'reset_odometry'))
                    message = rosmessage('std_msgs/Empty'); 
                    
                else
                    returnCode = -1;
                    
                end
            end
        case {'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', ...
                'arm_5_joint', 'gripper_1_joint'}
            if(strcmp(publisher(3), 'command'))
                message = rosmessage('std_msgs/Float64');
                
            elseif(strcmp(publisher(3), 'enable'))
                message = rosmessage('arbotix_msgs/Enable');
                
            elseif(strcmp(publisher(3), 'relax'))
                message = rosmessage('arbotix_msgs/Relax');
                
            elseif(strcmp(publisher(3), 'set_speed'))
                message = rosmessage('arbotix_msgs/SetSpeed');
                
            else
                returnCode = -1;
                
            end
        case 'vrep'
            switch (char(publisher(3)))
                case 'status_bar_message'
                    message = rosmessage('std_msgs/String');      
                    
                case 'aux_console'
                    switch(char(publisher(4)))
                        case 'print'
                            message = rosmessage('std_msgs/String');  
                            
                        otherwise
                            message = rosmessage('std_msgs/Empty');
                            
                    end
                case 'pause_communication'
                    message = rosmessage('std_msgs/Bool');
                    
                otherwise
                    returnCode = -1;
                    
            end 
        otherwise
            returnCode = -1;
            
    end
end