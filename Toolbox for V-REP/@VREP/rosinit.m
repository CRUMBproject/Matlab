function [returnCode] = rosinit(robot, v_ip_addr, v_port, r_ip_addr, r_port)
%ROSINIT Initializes the connection between MATLAB and V-REP and/or between
%MATLAB and ROS (depending of selected operating mode).
%
%   [returnCode] = rosinit(robot, v_ip_addr, v_port, r_ip_addr, r_port)
%
%   'returnCode' will allways be set to '0' (no errors). In case of error
%   program will end.

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
    
    % Displayin the header
    disp('####################################################################################################');
    disp('Starting....');
    ver robotics;
    disp('----------------------------------------------------------------------------------------------------');
    
    % REAL ROBOT [ROS]
    if(robot.operating_mode == 1 || robot.operating_mode == 2)
        % In this case, we will use the Robotics System Toolbox directly
        rosinit(r_ip_addr, r_port);  
        robot.r_ip_addr = r_ip_addr;
        robot.r_port    = r_port;
    end
    
    % V-REP
    if(robot.operating_mode == 0 || robot.operating_mode == 2)
        % Setting connection parameters
        robot.v_ip_addr  = v_ip_addr;
        robot.v_port     = v_port;
        
        % Automatic search for the correct library according to computer
        robot.vrep       = remApi(strcat('remoteApi_', computer));
        
        % Ending all previous comminicating threads
        robot.vrep.simxFinish(-1);
        
        % Trying to connect
        robot.v_clientID = robot.vrep.simxStart(v_ip_addr, v_port, true, true, 5000, 5);
                
        if (robot.v_clientID > -1) % Connection OK
            % Notificating the succesful connection in V-REP
            robot.vrep.simxAddStatusbarMessage(robot.v_clientID,...
                'Connected from MATLAB', robot.vrep.simx_opmode_oneshot);
            
            % Acquiring the necessary handlers
            robot.acquireVREPHandlers();
            
            % Setting VREP topics and services
            robot.setVREPTopics();
            robot.setVREPServices();
            
            % Initializing two control variables
            robot.header_seq_number = 0;            
            robot.hokuyo_last_measure = 0;
            
            % Initializing some streamings
            robot.vrep.simxGetVisionSensorImage2(robot.v_clientID,...
                robot.v_handles(2, 1), 0, robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetVisionSensorImage2(robot.v_clientID,...
                robot.v_handles(1, 1), 0, robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetVisionSensorImage2(robot.v_clientID,...
                robot.v_handles(2, 1), 1, robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetVisionSensorImage2(robot.v_clientID,...
                robot.v_handles(1, 1), 1, robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetStringSignal(robot.v_clientID,...
                'Turtlebot2_WidowX_kobuki_quaternion_vel_accel',  robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetStringSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_kobuki_sensor_state', robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetStringSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_widowx_control', robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetFloatSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_simulation_time', robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetStringSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_kobuki_controller_info', robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetStringSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_hokuyo_data', robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetStringSignal(robot.v_clientID, ...
                'Turtlebot2_WidowX_joint_states', robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetObjectPosition(robot.v_clientID,...
                robot.v_handles(9, 1), -1, robot.vrep.simx_opmode_streaming);
            robot.vrep.simxGetObjectOrientation(robot.v_clientID,...
                robot.v_handles(9, 1), -1, robot.vrep.simx_opmode_streaming);

            fprintf('Ping time: %d msecs.\n', robot.getPingTime());
            disp('----------------------------------------------------------------------------------------------------');

            % Sync pause 
            pause(0.5);
            
            disp('[OK]');
            disp('####################################################################################################');
            
            % No errors
            returnCode = 0;
            
        else % Connection fail
            % Throwing an error (this will end the current program)
            error('Cannot connect to V-REP, check IP and port and restart simulation');
        end
    end
end