function [header, returnCode] = getMessageHeader(robot)
%GETMESSAGEHEADER gets a generic message of type 'header' based on robot's
%context.
%
%   [header, returnCode] = getMessageHeader(robot)
%
%   'returnCode' will be always set to '0' (no errors).

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
%   uint32 Seq
%   time Stamp
%   string FrameId

    % Getting the message
    header = rosmessage('std_msgs/Header');
    
    % Setting the sequence number from the robot
    header.Seq = robot.header_seq_number;
        
    % Epoch time at the creation of the message
    header.Stamp.Sec = int32((datenum(clock) - datenum([1970  01  01  ...
        00  00  00])) * 86400);

    % Time nanoseconds will be always set to 0
    header.Stamp.Nsec = 0;

    % Default frame ID is empty
    header.FrameId = '';
      
    % Updating the robot's header sequence number
    robot.header_seq_number = robot.header_seq_number + 1;
    
    % Return code will be always '0'
    returnCode = 0;
end