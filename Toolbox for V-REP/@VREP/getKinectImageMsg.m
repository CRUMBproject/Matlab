function [v_msg, returnCode] = getKinectImageMsg(robot, depth, color, compressed)
%GETKINECTIMAGEMSG gets a message with current depth/rgb data from Kinect
%sensor
%
%   [v_msg, returnCode] = getKinectImageMsg(robot, depth, color, compressed)
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

    % Checking if depth or RGB
    if(depth)
        % Depth
        handle = robot.v_handles(2, 1); 
        
    else
        % RGB
        handle = robot.v_handles(1, 1);        
    end
    
    % Receiving the image from V-REP
    [result, resolution, imageData] = ...
          robot.vrep.simxGetVisionSensorImage2(robot.v_clientID,...
          handle, color, robot.vrep.simx_opmode_buffer);

    % Checking if compressed or not
    if(compressed)
        % Creating the message and setting the format
        v_msg        = rosmessage('sensor_msgs/CompressedImage');
        v_msg.Format = 'rgb8; png compressed bgr8';
    else
        % Creating the message
        v_msg = rosmessage('sensor_msgs/Image');
        
        % Checking if color or mono
        if(color == 0)
            v_msg.Encoding    = 'rgb8';
	        v_msg.Step        = resolution(1) * 3;
        else
            v_msg.Encoding    = 'mono8';
	        v_msg.Step        = resolution(1);
        end
        
        % Setting the height and width of the image
        v_msg.Height = resolution(2);
        v_msg.Width  = resolution(1);
        
        % Not is bigendian(by default
        v_msg.IsBigendian = 0;
        
        % Writting image to the message
        writeImage(v_msg, imageData); 
    end  
    
    % Getting a new message header
    v_msg.Header = robot.getMessageHeader();
            
    % Checking for errors
    returnCode = 0;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end
end