function returnCode = rosservice(robot, arg0)
%ROSSERVICE retrieves information about ROS services. Only 'list'
%information is implemented
%
%   returnCode = rosservice(robot, 'list')
%
%   'returnCode' will be set to '0' if operation ends succesfully or to
%   '-1' in case of error

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

    % ROS and MIXED
    if(robot.operating_mode == 1 || robot.operating_mode == 2)
        if(strcmp(arg0, 'list'))
            disp('------ REAL ROBOT -------');
                rosservice(arg0);
                returnCode = 0;
            disp('-------------------------'); 
        else
            returnCode = -1;
        end
     end
    
    % V-REP and MIXED
    if(robot.operating_mode == 0 ||robot.operating_mode == 2)
        if(strcmp(arg0, 'list'))
            disp('----- VIRTUAL ROBOT -----');
            for k = 1 : size(robot.v_services)
                disp(robot.v_services(k));
            end
            returnCode = 0;
            disp('-------------------------');
        else
            returnCode = -1;
        end
    end
end