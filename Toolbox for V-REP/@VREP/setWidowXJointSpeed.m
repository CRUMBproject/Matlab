function [returnCode] = setWidowXJointSpeed(robot, joint, speed)
%SETWIDOWXJOINTSPEED sets the maximum angular speed for a defined joint.
%
%   [returnCode] = setWidowXJointSpeed(robot, joint, speed)
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

    % Initializating error variable
    error = 0;
    
    % Handles base for WidowX's jouints
    handles_base = 3;
    
    % Searching for the desired joint
    switch (joint)
        case 'arm'
            handle = handles_base; 
            resetDynamics = 1;
        
        case 'shoulder'
            handle = handles_base + 1; 
            resetDynamics = 2;
           
        case 'biceps'
            handle = handles_base + 2;            
            resetDynamics = 4;
            
        case 'forearm'
            handle = handles_base + 3; 
            resetDynamics = 8;
               
        case 'wrist'
            handle = handles_base + 4;
            resetDynamics = 16;
               
        case 'gripper'
            handle = handles_base + 5; 
            resetDynamics = 32;
       
        otherwise
            error = 1;
    end
    
    if (error == 0)     
        % Setting the new max speed in V-REP
        result1 = robot.vrep.simxSetObjectFloatParameter(robot.v_clientID, ...
            robot.v_handles(handle, 1), 2017, speed, robot.vrep.simx_opmode_oneshot);  
    
        % Setting the joint which have to be reseted (in dynamics engine, see
        % V-REP implementation)
        result2 = robot.vrep.simxSetIntegerSignal(robot.v_clientID, ...
            'Turtlebot2_WidowX_widowx_reset_dynamics', ...
            resetDynamics, robot.vrep.simx_opmode_oneshot); 
    end

    % Checking for errors
    returnCode = error;
    if(result1 ~= robot.vrep.simx_return_ok || result2 ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end 
end