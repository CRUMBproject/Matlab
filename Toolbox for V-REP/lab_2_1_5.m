% #########################################################################
%                  TURTLEBOT+WIDOWX LAB 2.1.5 APPLICATION
% #########################################################################
% -------------------------------------------------------------------------
% This program moves the robot from the initial pose to one of the green
% tables that are placed on the lab. Then the robot takes the Rubik's cube
% that is on the table. Finally, the robot moves to the second green table
% and puts the Rubik's cube on it.
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

% Path with API functions
addpath('api');
addpath('tools');

% ----------------- Program parameters  and variables ---------------------
% V-REP
v_ip_addr = '127.0.0.1';
v_port = 19999;
    
% Real Turtle
r_ip_addr = '10.42.0.17';
r_port = 11311;

% Behaviour
defaultLinearVelocity        = 0.2;
defaultFastAngularVelocity   = pi;
defaultNormalAngularVelocity = pi / 2; 
defaultSlowAngularVelocity   = pi / 64;
% -------------------------------------------------------------------------

% Creating an instance of class 'VREP' and starting the simulation
myTurtle = VREP('vrep');
myTurtle.rosinit(v_ip_addr, v_port, r_ip_addr, r_port);



% ------------- Publishers, subscribers and services ----------------------
% Creating the publishers
led1_pub               = myTurtle.rospublisher('/mobile_base/commands/led1');
led2_pub               = myTurtle.rospublisher('/mobile_base/commands/led2');
kob_velocity_pub       = myTurtle.rospublisher('/mobile_base/commands/velocity');
resetodom_pub          = myTurtle.rospublisher('/mobile_base/commands/reset_odometry');
wid_1_pub              = myTurtle.rospublisher('/arm_1_joint/command');
wid_2_pub              = myTurtle.rospublisher('/arm_2_joint/command');
wid_3_pub              = myTurtle.rospublisher('/arm_3_joint/command');
wid_4_pub              = myTurtle.rospublisher('/arm_4_joint/command');
wid_5_pub              = myTurtle.rospublisher('/arm_5_joint/command');
wid_gripper_pub        = myTurtle.rospublisher('/gripper_1_joint/command');
vrep_st_bar_pub        = myTurtle.rospublisher('/vrep/status_bar_message');
vrep_console_open_pub  = myTurtle.rospublisher('/vrep/aux_console/create');
vrep_console_print_pub = myTurtle.rospublisher('/vrep/aux_console/print');

% Creating the subscribers
odom_sub         = myTurtle.rossubscriber('/odom');
sensors_sub      = myTurtle.rossubscriber('/mobile_base/sensors/core');
imu_sub          = myTurtle.rossubscriber('/mobile_base/sensors/imu_data');
sim_time_sub     = myTurtle.rossubscriber('/simulation/sim_time');

% Creating the services
wid_2_s_serv = myTurtle.rossvcclient('/arm_2_joint/set_speed');

wid_1_r_serv = myTurtle.rossvcclient('/arm_1_joint/relax');
wid_2_r_serv = myTurtle.rossvcclient('/arm_2_joint/relax');
wid_3_r_serv = myTurtle.rossvcclient('/arm_3_joint/relax');
wid_4_r_serv = myTurtle.rossvcclient('/arm_4_joint/relax');
wid_5_r_serv = myTurtle.rossvcclient('/arm_5_joint/relax');

% Creating the publishers's messages
velocity_msg      = myTurtle.rosmessage(kob_velocity_pub);
ledmsg            = myTurtle.rosmessage(led1_pub);
wid_joint_cmd_msg = myTurtle.rosmessage(wid_1_pub);
console_print_msg = myTurtle.rosmessage(vrep_console_print_pub);

% Creating the services's messages
wid_speed_msg     = myTurtle.rosmessage(wid_2_s_serv);
wid_relax_msg     = myTurtle.rosmessage(wid_2_r_serv);
% -------------------------------------------------------------------------

% Creating a V-REP's status bar notification
status_bar_msg = myTurtle.rosmessage(vrep_st_bar_pub);
status_bar_msg.Data = 'Program: Lab 2.1.5 WidowX Testing';
myTurtle.send(vrep_st_bar_pub, status_bar_msg);

% ------------ V-REP's auxiliar console initialization --------------------
console_open_msg = myTurtle.rosmessage(vrep_console_open_pub);
myTurtle.send(vrep_console_open_pub, console_open_msg);

sim_time_msg = myTurtle.receive(sim_time_sub);
initial_sim_time = sim_time_msg.Data;

disp(strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Initialized OK'));
console_print_msg.Data = strcat('[', num2str(sim_time_msg.Data,...
    '%.3f'),'] Initialized OK');    
myTurtle.send(vrep_console_print_pub, console_print_msg);
% -------------------------------------------------------------------------

% LEDs initialization
ledmsg.Value = ledmsg.ORANGE;
myTurtle.send(led1_pub, ledmsg);    % LED 1

ledmsg.Value = ledmsg.RED;
myTurtle.send(led2_pub, ledmsg);    % LED 2

% ------------ Turtle moving to the first green table ---------------------
sim_time_msg = myTurtle.receive(sim_time_sub);
disp(strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Moving to the first green table...'));
console_print_msg.Data = strcat('[', ...
    num2str(sim_time_msg.Data, '%.3f'),'] Moving to the first green table...');    
myTurtle.send(vrep_console_print_pub, console_print_msg);

% Reading the very first x coordinate
odom_msg  = myTurtle.receive(odom_sub);       
x         = getKobukiPoseAndOrientation(odom_msg);

while x < 2.915 % Meters
    % Reading the current pose and orientation
    odom_msg      = myTurtle.receive(odom_sub);      
    [x, y, theta] = getKobukiPoseAndOrientation(odom_msg);
    
    % We show the current pose in V-REP's console
    console_print_msg.Data = strcat('X: ', num2str(x),', Y: ', num2str(y),...
        ', Theta: ', num2str(theta));    
    myTurtle.send(vrep_console_print_pub, console_print_msg);

    % Receiving the sensors core message
    sens_msg = myTurtle.receive(sensors_sub);
    
    % Check for obstacle
    if(sens_msg.Bumper ~= 0 || sens_msg.Cliff ~= 0)
        % Obstacle    
        velocity_msg.Linear.X  = -0.2;
        velocity_msg.Angular.Z = 0;  
        myTurtle.send(kob_velocity_pub, velocity_msg);       
        myTurtle.pause(1.5);
        
        velocity_msg.Linear.X  = 0;
        velocity_msg.Angular.Z = defaultNormalAngularVelocity; 
        myTurtle.send(kob_velocity_pub, velocity_msg);        
        myTurtle.pause(1.5);        
    else
        % No obstacle
        velocity_msg.Linear.X  = defaultLinearVelocity;
        velocity_msg.Angular.Z = -0.1 * theta;        
        myTurtle.send(kob_velocity_pub, velocity_msg);       
    end
end
% User stopping notification
sim_time_msg = myTurtle.receive(sim_time_sub);
disp(strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Stopping the turtle...'));
console_print_msg.Data = strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Stopping the turtle...');    
myTurtle.send(vrep_console_print_pub, console_print_msg);

% Stopping the turtle
velocity_msg.Linear.X  = 0;
velocity_msg.Angular.Z = 0; 
myTurtle.send(kob_velocity_pub, velocity_msg);
myTurtle.pause(1);
% -------------------------------------------------------------------------

% ------------------ Trying to take the cube ------------------------------
% User's notification
sim_time_msg = myTurtle.receive(sim_time_sub);
disp(strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Trying to take the cube...'));
console_print_msg.Data = strcat('[', num2str(sim_time_msg.Data, '%.3f'),...
    '] Trying to take the cube...');
myTurtle.send(vrep_console_print_pub, console_print_msg);

% Biceps
wid_joint_cmd_msg.Data = pi / 2;        % 90 deg.
myTurtle.send(wid_3_pub, wid_joint_cmd_msg);
myTurtle.pause(1.5);

% Forearm
wid_joint_cmd_msg.Data = - (pi / 2);    % -90 deg.
myTurtle.send(wid_4_pub, wid_joint_cmd_msg);
myTurtle.pause(1.5);

% Arm
wid_joint_cmd_msg.Data = 0;             % 0 deg.
myTurtle.send(wid_1_pub, wid_joint_cmd_msg);
myTurtle.pause(1.5);

% Shoulder
wid_joint_cmd_msg.Data = deg2rad(65);   % 65 deg.
wid_speed_msg.Speed = deg2rad(10);
myTurtle.call(wid_2_s_serv, wid_speed_msg);
myTurtle.send(wid_2_pub, wid_joint_cmd_msg);
myTurtle.pause(5);
wid_speed_msg.Speed = deg2rad(100);     % 100 deg.
myTurtle.call(wid_2_s_serv, wid_speed_msg);

% Wrist
wid_joint_cmd_msg.Data = 0;             % 0 deg.
myTurtle.send(wid_5_pub, wid_joint_cmd_msg);
myTurtle.pause(1.5);

% Gripper
wid_joint_cmd_msg.Data = 1;             % 44 percent opened.
myTurtle.send(wid_gripper_pub, wid_joint_cmd_msg);
myTurtle.pause(2);

% ---------------- 'Transport' position of WidowX -------------------------
% Shoulder
wid_joint_cmd_msg.Data = deg2rad(-15);
myTurtle.send(wid_2_pub,wid_joint_cmd_msg);

% Forearm
wid_joint_cmd_msg.Data = pi / 2;
myTurtle.send(wid_4_pub, wid_joint_cmd_msg);

% Biceps
wid_joint_cmd_msg.Data = -(pi / 2);
myTurtle.send(wid_3_pub, wid_joint_cmd_msg);

% Wrist
wid_joint_cmd_msg.Data = 0;
myTurtle.send(wid_5_pub, wid_joint_cmd_msg);

% Shoulder
wid_joint_cmd_msg.Data = -(pi / 2);
myTurtle.send(wid_2_pub,wid_joint_cmd_msg);
myTurtle.pause(1);

% ----------------- Turtlebot orientation changing ------------------------
% User's notification
sim_time_msg = myTurtle.receive(sim_time_sub);
disp(strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Turning the turtle 180 degrees...'));
console_print_msg.Data = strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Turning the turtle 180 degrees...');    
myTurtle.send(vrep_console_print_pub, console_print_msg);

% Reading the first orientation
odom_msg      = myTurtle.receive(odom_sub);
[~, ~, theta] = getKobukiPoseAndOrientation(odom_msg);
theta = thetaTo360(theta);

while theta > 180.5 || theta < 179.5
     
     % Reading the current orientation
     odom_msg      = myTurtle.receive(odom_sub);
     [~, ~, theta] = getKobukiPoseAndOrientation(odom_msg);
     theta = thetaTo360(theta);
         
     if theta > 180.5
         if(abs(theta - 180.5) > 15)
             angularVelocity = -defaultNormalAngularVelocity;  
         else
             angularVelocity = -defaultSlowAngularVelocity;
         end
     else
         if(abs(theta - 179.5) > 15)
             angularVelocity = defaultNormalAngularVelocity;  
         else
             angularVelocity = defaultSlowAngularVelocity;
         end
     end
     
     velocity_msg.Linear.X  = 0;
     velocity_msg.Angular.Z = angularVelocity;  
     myTurtle.send(kob_velocity_pub, velocity_msg);
      
     % User's pose notification
     console_print_msg.Data = strcat('Pose -> X: ', num2str(x),', Y: ', num2str(y),...
         ', Theta: ', num2str(theta)); 
     myTurtle.send(vrep_console_print_pub, console_print_msg);      
end
% Stopping the turtle
velocity_msg.Linear.X  = 0;
velocity_msg.Angular.Z = 0; 
myTurtle.send(kob_velocity_pub, velocity_msg);

% User stopping notification
sim_time_msg = myTurtle.receive(sim_time_sub);
disp(strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Stopping the turtle...'));
console_print_msg.Data = strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Stopping the turtle...');    
myTurtle.send(vrep_console_print_pub, console_print_msg);


% -------------------------------------------------------------------------

% ----------- Turtlebot's moving to the second green table ----------------
% User notification
sim_time_msg = myTurtle.receive(sim_time_sub);
disp(strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Moving to the second green table...'));
console_print_msg.Data = strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Moving to the second green table...');    
myTurtle.send(vrep_console_print_pub, console_print_msg);

% First pose and orientation acquirement
odom_msg = myTurtle.receive(odom_sub);
[x, y, theta] = getKobukiPoseAndOrientation(odom_msg);

while x > -3
    % Current pose and orientation acquirement and notification
    odom_msg = myTurtle.receive(odom_sub);    
    [x, y, theta] = getKobukiPoseAndOrientation(odom_msg);
    theta = thetaTo360(theta);
    console_print_msg.Data = strcat('X: ', num2str(x),', Y: ', num2str(y),...
        ', Theta: ', num2str(theta));    
    myTurtle.send(vrep_console_print_pub, console_print_msg);

    sens_msg = myTurtle.receive(sensors_sub);
    
    % Checking for obstacle
    if(sens_msg.Bumper ~= 0 || sens_msg.Cliff ~= 0)
            
        velocity_msg.Linear.X  = -defaultNormalAngularVelocity;
        velocity_msg.Angular.Z = 0;
        
        myTurtle.send(kob_velocity_pub, velocity_msg);
        
        myTurtle.pause(1.5);
        
        velocity_msg.Linear.X  = 0;
        velocity_msg.Angular.Z = defaultNormalAngularVelocity;
        
        myTurtle.send(kob_velocity_pub, velocity_msg);
        
        myTurtle.pause(1.5);
    else
        velocity_msg.Linear.X  = defaultLinearVelocity;
        velocity_msg.Angular.Z = -0.1 * (theta - 180); 
        myTurtle.send(kob_velocity_pub, velocity_msg);        
    end    
end

% Stopping the turtle
velocity_msg.Linear.X  = 0;
velocity_msg.Angular.Z = 0; 
myTurtle.send(kob_velocity_pub, velocity_msg);
myTurtle.pause(1);

% ------------ Placing the Rubik's cube to the second table ---------------
% User's notification
sim_time_msg = myTurtle.receive(sim_time_sub);
disp(strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Dropping the cube on the table...'));
console_print_msg.Data = strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Dropping the cube on the table...');    
myTurtle.send(vrep_console_print_pub, console_print_msg);

% Shoulder
wid_joint_cmd_msg.Data = deg2rad(40);
myTurtle.send(wid_2_pub, wid_joint_cmd_msg);

% Biceps
wid_joint_cmd_msg.Data = deg2rad(50);
myTurtle.send(wid_3_pub, wid_joint_cmd_msg);

% Forearm
wid_joint_cmd_msg.Data = -(pi / 2);
myTurtle.send(wid_4_pub, wid_joint_cmd_msg);
myTurtle.pause(2);

% Gripper
wid_joint_cmd_msg.Data = 0;
myTurtle.send(wid_gripper_pub, wid_joint_cmd_msg);
myTurtle.pause(2);

% ---------- Homming of WidowX and relaxing of the joints -----------------
% User's notification of homming
sim_time_msg = myTurtle.receive(sim_time_sub);
console_print_msg.Data = strcat('[', num2str(sim_time_msg.Data, '%.3f'),...
    '] Homming WidowX...');    
myTurtle.send(vrep_console_print_pub, console_print_msg);
disp(console_print_msg.Data);

wid_joint_cmd_msg.Data = 0;

% Shoulder
myTurtle.send(wid_2_pub, wid_joint_cmd_msg);

% Forearm
myTurtle.send(wid_4_pub, wid_joint_cmd_msg);

% Biceps
myTurtle.send(wid_3_pub, wid_joint_cmd_msg);
myTurtle.pause(2);

% Calling to the relaxing services of each joint
myTurtle.call(wid_1_r_serv, wid_relax_msg);
myTurtle.call(wid_2_r_serv, wid_relax_msg);
myTurtle.call(wid_3_r_serv, wid_relax_msg);
myTurtle.call(wid_4_r_serv, wid_relax_msg);
myTurtle.call(wid_5_r_serv, wid_relax_msg);

% ------------------------ Program Ending ---------------------------------
% Notifying the program's ending to the user
sim_time_msg = myTurtle.receive(sim_time_sub);
disp(strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Program ended.'));
console_print_msg.Data = strcat('[', num2str(sim_time_msg.Data, '%.3f'),'] Program ended.');    
myTurtle.send(vrep_console_print_pub, console_print_msg);

% Displaying the simulation's elapsed time
sim_time_msg = myTurtle.receive(sim_time_sub);
elapsed_time =  sim_time_msg.Data -initial_sim_time;
disp(strcat('Elapsed simulation time:', num2str(elapsed_time),' seconds.'));
console_print_msg.Data = strcat('Elapsed simulation time:', num2str(elapsed_time),' seconds.');    
myTurtle.send(vrep_console_print_pub, console_print_msg);


% Closing the connection
myTurtle.rosshutdown;
% -------------------------------------------------------------------------