% #########################################################################
%                TURTLEBOT+WIDOWX APPLICATION EXAMPLE
% #########################################################################
% -------------------------------------------------------------------------
% This file is an example on how to use the remote connection to V-REP or
% to real robot from MATLAB. Simply you can use it for building your own 
% program.
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

% Path with API functions and tools
addpath('api');
addpath('tools');

% ------------------------ Program parameters -----------------------------
% V-REP
v_ip_addr = '127.0.0.1';    % V-REP is in this machine
v_port = 19999;             % Port is configured in V-REP model
    
% Real Turtle
r_ip_addr = '10.42.0.17';   % Check IP on real turtle's configuration 
r_port = 11311;             % Check port on real turtle's configuration

% Behaviour
defaultLinearVelocity  = 0.2;    % Meters per second
defaultAngularVelocity = pi / 2; % Radians per second
% -------------------------------------------------------------------------

% Creating an instance of class 'VREP' and starting the simulation
myTurtle = VREP('vrep');
myTurtle.rosinit(v_ip_addr, v_port, r_ip_addr, r_port);

% ------------- Publishers, subscribers and services ----------------------
%       SEE MANUAL FOR LIST OF SUPPORTED TOPICS AND SERVICES

% Creating the publishers
kob_velocity_pub       = myTurtle.rospublisher('/mobile_base/commands/velocity');

vrep_st_bar_pub        = myTurtle.rospublisher('/vrep/status_bar_message');
vrep_console_open_pub  = myTurtle.rospublisher('/vrep/aux_console/create');
vrep_console_print_pub = myTurtle.rospublisher('/vrep/aux_console/print');

% Creating the subscribers
odom_sub         = myTurtle.rossubscriber('/odom');
sim_time_sub     = myTurtle.rossubscriber('/simulation/sim_time');

% Creating the publishers's messages
velocity_msg      = myTurtle.rosmessage(kob_velocity_pub);
status_bar_msg    = myTurtle.rosmessage(vrep_st_bar_pub);
console_open_msg  = myTurtle.rosmessage(vrep_console_open_pub);
console_print_msg = myTurtle.rosmessage(vrep_console_print_pub);
% -------------------------------------------------------------------------

% Creating a V-REP's status bar notification
status_bar_msg.Data = 'Program: MY PROGRAM';
myTurtle.send(vrep_st_bar_pub, status_bar_msg);

% ------------ V-REP's auxiliar console initialization --------------------
myTurtle.send(vrep_console_open_pub, console_open_msg);

disp('Initialized OK.');
console_print_msg.Data = 'Initialized OK.';    
myTurtle.send(vrep_console_print_pub, console_print_msg);
% -------------------------------------------------------------------------

% /////////////////////////////////////////////////////////////////////////
% -------------------- HERE GOES YOUR ALGORITHM ---------------------------

% Notification
disp('Moving the turtle for 3 seconds...');
console_print_msg.Data = strcat('Moving the turtle for 3 seconds...');    
myTurtle.send(vrep_console_print_pub, console_print_msg);

% Getting the starting simulation time
sim_time_msg    = myTurtle.receive(sim_time_sub);
startingSimTime = sim_time_msg.Data;
currentSimTime  = startingSimTime;

while(currentSimTime - startingSimTime < 3) % 3 seconds
    % Getting odometry data
    odom_msg      = myTurtle.receive(odom_sub);       
    [x, y, theta] = getKobukiPoseAndOrientation(odom_msg);
    
    % Getting the current simulation time
    sim_time_msg   = myTurtle.receive(sim_time_sub);
    currentSimTime = sim_time_msg.Data;
    
    % Showing the current pose in V-REP's console
    console_print_msg.Data = strcat('X: ', num2str(x),', Y: ', num2str(y),...
        ', Theta: ', num2str(theta));    
    myTurtle.send(vrep_console_print_pub, console_print_msg);
    
    % Sending velocity command to the turtle
    velocity_msg.Linear.X  = defaultLinearVelocity;    
    velocity_msg.Angular.Z = 0;
    myTurtle.send(kob_velocity_pub, velocity_msg);       
end 

% Notification
disp('Stopping the turtle...');
console_print_msg.Data = strcat('Stopping the turtle...');    
myTurtle.send(vrep_console_print_pub, console_print_msg);

% Stopping the turtle
velocity_msg.Linear.X  = 0;
velocity_msg.Angular.Z = 0; 
myTurtle.send(kob_velocity_pub, velocity_msg);

% Waiting for 1 second (in simulation time)
myTurtle.pause(1);

%\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

% Closing the connection
myTurtle.rosshutdown;