function [out_1, returnCode] = rostopic(robot, arg0, arg1)
%ROSTOPIC retrieves information for a specific topic.
%
%   [out_1, returnCode] = rostopic(robot, 'list')
%   [out_1, returnCode] = rostopic(robot, 'type', topicname)
%
%   'returnCode' will always be set to '0' (No errors)

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

    % Initialization of 'returnCode'
    returnCode = -1;
    
    % ROS and MIXED
    if(robot.operating_mode == 1 ||robot.operating_mode == 2)
        disp('------ REAL ROBOT -------');
        if nargin == 2            
            rostopic(arg0);
            returnCode = 0;
            
        elseif nargin == 3
            rostopic(arg0, arg1);
            returnCode = 0;
            
        end
        disp('-------------------------');
    end
    
    % V-REP and MIXED
    if(robot.operating_mode == 0 || robot.operating_mode == 2)
        disp('----- VIRTUAL ROBOT -----');
        if(strcmp(arg0, 'list'))
            out_1 = robot.v_topics;
            returnCode = 0;
            
        elseif (strcmp(arg0, 'type'))
            out_1 = robot.rostopicTypeVREP(arg1);
            returnCode = 0;
            
        end
        disp('-------------------------');
    end
 end