function [v_msg, returnCode] = getKobukiOdometryMsg(robot)
%GETKOBUKIODOMETRYMSG gets the Kobuiki's odometry message.
%
%   [v_msg, returnCode] = getKobukiOdometryMsg(robot)
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

% Structure of nav_msgs/Odometry:
%   Header header
%   string child_frame_id
%   geometry_msgs/PoseWithCovariance pose
%   geometry_msgs/TwistWithCovariance twist

    % Getting the odometry data from V-REP
    [result, poseQuaternionData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
        'Turtlebot2_WidowX_kobuki_quaternion_vel_accel',  robot.vrep.simx_opmode_buffer);
    
    % Unpacking the odometry data
    poseQuaternionVel    = robot.vrep.simxUnpackFloats(poseQuaternionData);    
    x                    = poseQuaternionVel(1);
    y                    = poseQuaternionVel(2);
    z                    = poseQuaternionVel(3);
    xq                   = poseQuaternionVel(4);
    yq                   = poseQuaternionVel(5);
    zq                   = poseQuaternionVel(6);
    wq                   = poseQuaternionVel(7);
    lin_vel_x            = poseQuaternionVel(8);
    lin_vel_y            = poseQuaternionVel(9);
    lin_vel_z            = poseQuaternionVel(10);
    ang_vel_x            = poseQuaternionVel(11);
    ang_vel_y            = poseQuaternionVel(12);
    ang_vel_z            = poseQuaternionVel(13);    
    
    % Creating the message
    v_msg = rosmessage('nav_msgs/Odometry');
    
    % Setting the child frame ID
    v_msg.ChildFrameId = 'base_footprint';
    
    % Getting the message's header
    v_msg.Header = robot.getMessageHeader();
    
    % Setting the header's frame ID
    v_msg.Header.FrameId = 'odom';
    
    % Setting the postion's fields of the message
    v_msg.Pose.Pose.Position.X = x;
    v_msg.Pose.Pose.Position.Y = y;
    v_msg.Pose.Pose.Position.Z = z;
    
    % Setting the orientation's fields of the message
    v_msg.Pose.Pose.Orientation.X = xq;
    v_msg.Pose.Pose.Orientation.Y = yq;
    v_msg.Pose.Pose.Orientation.Z = zq;
    v_msg.Pose.Pose.Orientation.W = wq;
    
    % Setting the pose covariance matrix
    v_msg.Pose.Covariance = ...
        [0.1 0 0 0 0 0 ...
         0 0.1 0 0 0 0 ...
         0 0 1.7976931348623157e+308 0 0 0 ...
         0 0 0 1.7976931348623157e+308 0 0 ...
         0 0 0 0 1.7976931348623157e+308 0 ...
         0 0 0 0 0 0.05]';
    
    % Setting the linear velocity fields
    v_msg.Twist.Twist.Linear.X  = lin_vel_x;
    v_msg.Twist.Twist.Linear.Y  = lin_vel_y;    
    v_msg.Twist.Twist.Linear.Z  = lin_vel_z;
    
    % Setting the angular velocity fields
    v_msg.Twist.Twist.Angular.X = ang_vel_x;
    v_msg.Twist.Twist.Angular.Y = ang_vel_y;    
    v_msg.Twist.Twist.Angular.Z = ang_vel_z;
    
    % Setting the twist's covariance matrix
    v_msg.Twist.Covariance = zeros(36, 1);  
    
    % Checking for errors
    returnCode = 0;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end 
end