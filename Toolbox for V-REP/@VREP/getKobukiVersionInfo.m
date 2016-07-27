function [v_msg, returnCode] = getKobukiVersionInfo(robot)
%GETKOBUKIVERSIONINFO gets the Kobuki's version info message based in real 
%robot version info. This info is defined in the V-REP model.
%
%   [v_msg, returnCode] = getKobukiVersionInfo(robot)
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
%   string hardware   # <major>.<minor>.<patch>
%   string firmware   # <major>.<minor>.<patch>
%   string software   # Still to decide how it will look
%   uint32[] udid
%   uint64 features
%   uint64 SMOOTH_MOVE_START    = 0000000000000001
%   uint64 GYROSCOPE_3D_DATA    = 0000000000000002

    % Getting the version info data from V-REP
    [result, versionInfoData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
        'Turtlebot2_WidowX_kobuki_version_info', robot.vrep.simx_opmode_oneshot_wait);
    
    % Unpacking data
    versionInfo = robot.vrep.simxUnpackFloats(versionInfoData);
 
    % Creating the message
    v_msg = rosmessage('kobuki_msgs/VersionInfo');
    
    % Setting the message fields
    v_msg.Hardware = strcat(num2str(versionInfo(1)),'.',num2str(versionInfo(2)), ...
        '.', num2str(versionInfo(3)));
    v_msg.Firmware = strcat(num2str(versionInfo(4)),'.',num2str(versionInfo(5)), ...
        '.', num2str(versionInfo(6)));
    v_msg.Software = strcat(num2str(versionInfo(7)),'.',num2str(versionInfo(8)), ...
        '.', num2str(versionInfo(9)));
    v_msg.Udid     = [versionInfo(10), versionInfo(11), versionInfo(12)];
    v_msg.Features = versionInfo(13);
     
    % Checking for errors
    returnCode = 0;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end    
end