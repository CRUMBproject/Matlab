function [v_msg, returnCode] = getKobukiImuMsg(robot)
%GETKOBUKIIMUMSG gets the Kobuki's IMU message.
%
%   [v_msg, returnCode] = getKobukiImuMsg(robot)
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

% Structure of the message:
%   std_msgs/Header header
%   geometry_msgs/Quaternion orientation
%   float64[9] orientation_covariance
%   geometry_msgs/Vector3 angular_velocity
%   float64[9] angular_velocity_covariance
%   geometry_msgs/Vector3 linear_acceleration
%   float64[9] linear_acceleration_covariance

    % Getting the data from V-REP
    [result, poseQuaternionData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
        'Turtlebot2_WidowX_kobuki_quaternion_vel_accel', robot.vrep.simx_opmode_buffer);
     
    % Unpacking the data
    poseQuaternionVelAcc = robot.vrep.simxUnpackFloats(poseQuaternionData);

    % Getting a new message header
    header = robot.getMessageHeader();
    
    % Setting the header's frame ID
    header.FrameId = 'gyro_link';
    
    % Getting the message
    v_msg = rosmessage('sensor_msgs/Imu');  
    
    % Setting the message header
    v_msg.Header = header;
    
    % Setting the quaternion fields
    v_msg.Orientation.X = poseQuaternionVelAcc(4);
    v_msg.Orientation.Y = poseQuaternionVelAcc(5);
    v_msg.Orientation.Z = poseQuaternionVelAcc(6);
    v_msg.Orientation.W = poseQuaternionVelAcc(7);    
    
    % Setting the angular acceleration fields
    v_msg.AngularVelocity.X = poseQuaternionVelAcc(11);
    v_msg.AngularVelocity.Y = poseQuaternionVelAcc(12);
    v_msg.AngularVelocity.Z = poseQuaternionVelAcc(13); 
    
    % Setting the linear acceleration fields
    v_msg.LinearAcceleration.X = poseQuaternionVelAcc(14);
    v_msg.LinearAcceleration.Y = poseQuaternionVelAcc(15);
    v_msg.LinearAcceleration.Z = poseQuaternionVelAcc(16);
    
    % Setting the covariances matrix fields
    v_msg.OrientationCovariance        = [1.7976931348623157e+308, 0, 0, 0,...
        1.7976931348623157e+308 0, 0, 0, 0.05];    
    v_msg.AngularVelocityCovariance    = [1.7976931348623157e+308, 0, 0, 0,...
        1.7976931348623157e+308 0, 0, 0, 0.05];    
    v_msg.LinearAccelerationCovariance = zeros(1, 9); 
    
    % Checking for errors
    returnCode = 0;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end 
end