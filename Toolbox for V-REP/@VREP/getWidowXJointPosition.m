function [position, returnCode] = getWidowXJointPosition(robot, joint)
%GETWIDOWXJOINTSPOSITION gets the position of a defined joint of WidowX.
%
%   [position, returnCode] = getWidowXJointPosition(robot, joint)
%
%   'returnCode' will be set to '0' if operation ends successfully, to '1'
%   if joint not recognized or to '-1' if exits other error.

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
    
    % Getting the joint states data
    [result, statesData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
            'Turtlebot2_WidowX_joint_states', robot.vrep.simx_opmode_buffer);

    % Unpacking the joint states data
    states = robot.vrep.simxUnpackFloats(statesData);
    
    % Initializating error variable
    error = 0;
    
    % Searching for the desired joint
    switch(joint)
        case 'arm'
            position = states(7);   
        
        case 'shoulder'
            position = states(10); 
   
        case 'biceps'
            position = states(13);   
        
        case 'forearm'
            position = states(16);   

        case 'wrist'
            position = states(19);    
               
        case 'gripper'
            position = states(22); 
            
        otherwise
            error = 1;
    end
    
    % Checking for errors
    returnCode = error;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end  
end