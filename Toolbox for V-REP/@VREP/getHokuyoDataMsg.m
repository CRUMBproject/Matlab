function [v_msg, returnCode] = getHokuyoDataMsg(robot)
%GETHOKUYODATAMSG receives Hokuyo data and creates the message
%
%   [v_msg, returnCode] = getHokuyoDataMsg(robot)
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
%   float32 angle_min
%   float32 angle_max
%   float32 angle_increment
%   float32 time_increment
%   float32 scan_time
%   float32 range_min
%   float32 range_max
%   float32[] ranges
%   float32[] intensities

    % Receiving the data from V-REP
    [returnCode, hokuyoDataString] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
        'Turtlebot2_WidowX_hokuyo_data', robot.vrep.simx_opmode_buffer);
    
    % Unpacking the data
    hokuyoData = robot.vrep.simxUnpackFloats(hokuyoDataString);
    
    % First elemento of data is the simulation time of the data capturing
    time = hokuyoData(1);
    
    % Establishing data in two-colum vector form
    XYpoints = vec2mat(hokuyoData(2:end),2);
    
    % Changing the sign of the coordinates (the sensor is face down)    
    XYpoints(:, 2) = -XYpoints(:, 2);   
       
    % Calculating the ranges (euclidean distances)
    ranges = sqrt(XYpoints(: ,1).^2 + XYpoints(:,2).^2);
    
    % Calculating the time increment for the message
    timeIncrement = time - robot.hokuyo_last_measure;
    
    % Saving the last time measure
    robot.hokuyo_last_measure = time;
    
    % Creating the message
    v_msg = rosmessage('sensor_msgs/LaserScan');
    
    % Getting the message header
    v_msg.Header = robot.getMessageHeader();
    
    % Setting some fixed parameters (according to sensor's datasheet)
    v_msg.RangeMin       = 0.06;    
    v_msg.RangeMax       = 4.095;
    v_msg.AngleMin       = -2.0944;    
    v_msg.AngleMax       = 2.0944;    
    v_msg.AngleIncrement = 0.006123962287641342;
    
    % Setting the time increment and the scan time (we assume they are the
    % same)
    v_msg.TimeIncrement = timeIncrement;   
    v_msg.ScanTime      = timeIncrement;    

    % Setting the ranges in the message
    v_msg.Ranges = ranges;
    
    % Intensities will be empty (not implemented in the sensor)
    v_msg.Intensities = [];
end