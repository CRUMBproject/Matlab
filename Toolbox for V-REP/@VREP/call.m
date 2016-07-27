function [out_1, out_2, out_3] = call(robot, in1, in2)
%CALL calls a service provided by a server
%   CALL in/out arguments depends on selected operating mode as listed 
%   below:  
%
%   {V-REP mode}
%       [response, returnCode] = CALL(robot, service)
%       [response, returnCode] = CALL(robot, service, request)
%   {ROS mode:
%       [response, returnCode] = CALL(robot, service)
%       [response, returnCode] = CALL(robot, service, request)
%   Mixed mode:
%       [response_real, response_vrep, returnCode] = ...
%           CALL(robot, service)
%       [response_real, response_vrep, returnCode] = ...
%           CALL(robot, service, request)
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

    % Switch on operating mode
    switch (robot.operating_mode) 
        case 0 % V-REP
            [out_2, out_1] = callVREPService(robot, in1.ServerName, in2);
            out_3 = []; 
        
        case 1 % ROS 
            switch nargin
                case 2
                    out_1 = call(in1);
                case 3
                    out_1 = call(in1, in2); 
            end
            out_2 = 0;      % Return code
        
        case 2 % MIXED
            switch nargin
                case 2
                    out_1 = call(in1);
                case 3
                    out_1 = call(in1, in2); 
            end             
            [out_3, out_2] = callVREPService(robot, in1.ServerName, in2);       
    end
end
