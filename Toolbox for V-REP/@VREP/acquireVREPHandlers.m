function [returnCode] = acquireVREPHandlers(robot)
%ACQUIREVREPHANDLERS Acquires some handles from V-REP
%   [returnCode] = ACQUIREVREPHANDLES(robot) aquires some handles
%   (specified by the implementation of this function) from V-REP and
%   stores them in the 'v_handles' property of object 'robot'.
%
%   'returnCode' will be set to '0' if operation ends successfully or to 
%   '-1' if exits at least one error.

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

% HANDLES:
%       1) Kinect_RGB
%       2) Kinect_Depth
%       3) WidowX's arm joint
%       4) WidowX's shoulder joint
%       5) WidowX's biceps joint
%       6) WidowX's forearm joint
%       7) WidowX's wrist joint
%       8) WidowX's gripper_1 joint
%       9) Kobuki's gyroscope

    % Number of object handles
    nHandles = 9;
    
    % Results from obtaining joints's handles
    h_result = zeros(nHandles, 1); 

    % Handles initialization
    robot.v_handles = zeros(nHandles, 1);

    % Kinect
    [h_result(1, 1), robot.v_handles(1, 1)] = robot.vrep.simxGetObjectHandle ...
        (robot.v_clientID, 'kinect_rgb',   robot.vrep.simx_opmode_oneshot_wait);
    [h_result(2, 1), robot.v_handles(2, 1)] = robot.vrep.simxGetObjectHandle ...
        (robot.v_clientID, 'kinect_depth', robot.vrep.simx_opmode_oneshot_wait);
    
    % WidowX joints
    [h_result(3, 1), robot.v_handles(3, 1)] = robot.vrep.simxGetObjectHandle ...
        (robot.v_clientID, 'arm_joint', robot.vrep.simx_opmode_oneshot_wait);
    [h_result(4, 1), robot.v_handles(4, 1)] = robot.vrep.simxGetObjectHandle ...
        (robot.v_clientID, 'shoulder_joint', robot.vrep.simx_opmode_oneshot_wait);
    [h_result(5, 1), robot.v_handles(5, 1)] = robot.vrep.simxGetObjectHandle ...
        (robot.v_clientID, 'biceps_joint', robot.vrep.simx_opmode_oneshot_wait);
    [h_result(6, 1), robot.v_handles(6, 1)] = robot.vrep.simxGetObjectHandle ...
        (robot.v_clientID, 'forearm_joint', robot.vrep.simx_opmode_oneshot_wait);
    [h_result(7, 1), robot.v_handles(7, 1)] = robot.vrep.simxGetObjectHandle ...
        (robot.v_clientID, 'wrist_joint', robot.vrep.simx_opmode_oneshot_wait);
    [h_result(8, 1), robot.v_handles(8, 1)] = robot.vrep.simxGetObjectHandle ...
        (robot.v_clientID, 'gripper_1_joint', robot.vrep.simx_opmode_oneshot_wait);
    
    % Kobuki's gyroscope
    [h_result(9, 1), robot.v_handles(9, 1)] = robot.vrep.simxGetObjectHandle ...
        (robot.v_clientID, 'gyro_link_respondable', robot.vrep.simx_opmode_oneshot_wait);
    
    % Checking for errors
    returnCode = 0;
    ok = robot.vrep.simx_return_ok;
    for i = 1:nHandles
        if(h_result(i, 1) ~= ok)
            returnCode = -1;
        end
    end
end