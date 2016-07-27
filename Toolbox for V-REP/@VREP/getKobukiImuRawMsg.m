function [v_msg, returnCode] = getKobukiImuRawMsg(robot)
%GETKOBUKIIMURAWMSG gets the Kobuki's IMU message. Depending of selected
%mode, a noise will be generated or not.
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

    % Getting data from V-REP     
    [result, poseQuaternionData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
        'Turtlebot2_WidowX_kobuki_quaternion_vel_accel',  robot.vrep.simx_opmode_buffer);
     
    % Unpacking data from V-REP
    poseQuaternionVelAcc = robot.vrep.simxUnpackFloats(poseQuaternionData);
    
    % If noise enabled, Gaussian noise is generated
    if(robot.noise_enabled == 1)
        noise_ang_vel_X = normrnd(0.0034, 0.0016);  
        noise_ang_vel_Y = normrnd(0.004, 0.0017);
        noise_ang_vel_Z = normrnd(-0.0091, 0.002);
    else
        noise_ang_vel_X = 0;  
        noise_ang_vel_Y = 0;
        noise_ang_vel_Z = 0;        
    end
    
    % Getting a new message header
    header = robot.getMessageHeader();
    
    % Setting the header's frame ID
    header.frame_id = 'gyro_link';
    
    % Creating the message
    v_msg = rosmessage('sensor_msgs/Imu');
    
    % Setting the message's fields
    v_msg.Header                       = header;    
    v_msg.Orientation.X                = 0;
    v_msg.Orientation.Y                = 0;
    v_msg.Orientation.Z                = 0;
    v_msg.Orientation.W                = 0;    
    v_msg.AngularVelocity.X            = poseQuaternionVelAcc(11) + noise_ang_vel_X;
    v_msg.AngularVelocity.Y            = poseQuaternionVelAcc(12) + noise_ang_vel_Y;
    v_msg.AngularVelocity.Z            = poseQuaternionVelAcc(13) + noise_ang_vel_Z;     
    v_msg.LinearAcceleration.X         = poseQuaternionVelAcc(14);
    v_msg.LinearAcceleration.Y         = poseQuaternionVelAcc(15);
    v_msg.LinearAcceleration.Z         = poseQuaternionVelAcc(16);    
    v_msg.OrientationCovariance        = zeros(1, 9);    
    v_msg.AngularVelocityCovariance    = zeros(1, 9);    
    v_msg.LinearAccelerationCovariance = zeros(1, 9);
    
    % Checking for errors
    returnCode = 0;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end 
end