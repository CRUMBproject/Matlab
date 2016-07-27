function [msg, returnCode] = rosmessage(robot, entity)
%ROSMESSAGE gets a default message for the specified publisher or service.
%
%   {VREP Mode}   [msg, returnCode] = ROSMESSAGE(robot, entity)
%   {REAL Mode}   [msg, returnCode] = ROSMESSAGE(robot, entity)
%   {MIXED Mode}  [msg, returnCode] = ROSMESSAGE(robot, entity)
%
%   'returnCode' will always be set to '0' (no errors).

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

    if robot.operating_mode == 0 % V-REP 
        if(isfield(entity, 'TopicName'))
            [msg, returnCode] = robot.getVREPMessage(entity.TopicName);            
        elseif(isfield(entity, 'ServerName'))
            [msg, returnCode] = robot.getVREPMessage(entity.ServerName);
        else
            returnCode = -1;
        end
        
    elseif (robot.operating_mode == 1 || robot.operating_mode == 2) % ROS and Mixed         
        msg = rosmessage(entity);  
        returnCode = 0;      % Return code     
        
    end
end
