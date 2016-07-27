function [returnCode] = setVREPServices(robot)
%SETVREPSERVICES sets the list of implemented services for the simulated
%robot.
%
%   [returnCode] = setVREPServices(robot)
%
%   'returnCode' will always set to '0' (no errors).

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
    
    % List of services
    services = ['/arm_1_joint/enable       ';...  
                '/arm_1_joint/relax        ';...
                '/arm_1_joint/set_speed    ';...
                '/arm_2_joint/enable       ';...
                '/arm_2_joint/relax        ';...
                '/arm_2_joint/set_speed    ';...
                '/arm_3_joint/enable       ';...
                '/arm_3_joint/relax        ';...
                '/arm_3_joint/set_speed    ';...
                '/arm_4_joint/enable       ';...
                '/arm_5_joint/relax        ';...
                '/arm_5_joint/set_speed    ';...
                '/gripper_1_joint/enable   ';...
                '/gripper_1_joint/set_speed';...
                '/gripper_1_joint/relax    '];
    
    % Converstion to string
    robot.v_services  = cellstr(services);
    
    % No errors
    returnCode = 0;
end