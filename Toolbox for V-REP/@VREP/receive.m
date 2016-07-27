function [out_1, out_2, out_3] = receive(robot, in_1, in_2, in_3)
%RECEIVE - Wait for new ROS message
%
%   This function waits for MATLAB to receive a topic message from the
%   specified subscriber(s), and returns it as v_msg and/or r_msg. If
%   no timeout is specified, timeout will be set to Inf.
%   
%   {VREP Mode}   [v_msg, returnCode]        = RECEIVE(robot, sub, timeout)
%   {REAL Mode}   [r_msg, returnCode]        = RECEIVE(robot, sub, timeout)
%   {MIXED Mode}  [r_msg, v_msg, returnCode] = RECEIVE(robot, r_sub, v_sub, timeout)
%
%   'returnCode' will allways be set to '0' (no errors). In case of error
%   program will end.

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
    
    % Switching operating mode
    switch (robot.operating_mode) 
        case 0 % V-REP
            % Checking if timeout is specified or not
            if(nargin < 3)
                timeout = Inf;
            else
                timeout = in_2;
            end
            
            % Receiving message from V-REP
            [out_1, errorVREP] = robot.receiveFromVREP(in_1.TopicName, timeout);
            
            % Checking for errors
            if(errorVREP ~= 0)
                if(errorVREP == 1)
                    error('RECEIVE: No events occurred');
                else
                    error('RECEIVE: Error in communication.');                   
                end               
            else
                % ReturnCode
                out_2 = 0;
            end
            
            out_3 = [];
        
        case 1 % ROS
            if(nargin < 3)
                timeout = Inf;
            else
                timeout = in_2;
            end
            out_1 = receive(in_1, timeout);
            out_2 = 0;      % Return code  
            out_3 = [];
        
        case 2 % Mixed
            if(nargin < 4) 
                timeout = Inf;
            else
                timeout = in_3;
            end
            out_1              = receive(in_1, timeout);
            [out_2, errorVREP] = robot.receiveFromVREP(in_2.TopicName, timeout);  
           
            % Checking for errors
            if(errorVREP ~= 0)
                if(errorVREP == 1)
                    error('RECEIVE: No events occurred');
                else
                    error('RECEIVE: Error in communication.');                   
                end               
            else
                % ReturnCode
                out_3 = 0;
            end            
    end   
end
