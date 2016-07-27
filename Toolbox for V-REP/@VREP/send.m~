function [returnCode] = send(robot, in_1, in_2, in_3, in_4)
%SEND publish ROS message to a topic
%
%   {V-REP mode} -> [returnCode] = send(robot, publisher, message)
%   {ROS mode}   -> [returnCode] = send(robot, publisher, message)
%   {MIXED mode} -> [returnCode] = send(robot, publisher_r, message_r, publisher_v, message_v)
%
%   'returnCode' will depend on called internal function

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

    switch (robot.operating_mode)
        case 0 % V-REP        
        returnCode = sendToVREP(robot, in_1.TopicName, in_2); 
        
        case 1 % ROS         
        send(in_1, in_2); 
        returnCode = 0;      % Return code          
        
        case 2 % MIXED        
        send(in_1, in_2); 
        returnCode = sendToVREP(robot, in_3.TopicName, in_4);         
    end 
end
