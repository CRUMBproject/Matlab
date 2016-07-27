function [returnCode] = manageAuxiliaryVREPConsole(robot, command, message)
%MANAGEAUXILIARYVREPCONSOLE manages the state of the auxiliary console in
%V-REP. Accepted commands are: 'print', 'clear', 'create', 'delete', 'show'
%and hide.
%
%   [returnCode] = manageAuxiliaryVREPConsole(robot, command, message)
%
%   'returnCode' will be set to '0' if operation ends successfully, to '1'
%   if command not recognized or to '-1' if exits other error.

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

    % Initializating error and result variables
    error  = 0;
    result = 0; 

    % Searching for command
    switch command
        case 'print'
            % Prints a line in the console
           [result] = robot.vrep.simxAuxiliaryConsolePrint(robot.v_clientID,...
               robot.v_console, sprintf('>%s\n', message),...
               robot.vrep.simx_opmode_oneshot_wait);
            
        case 'clear'
            % Clears the current text in the console
            [result] = robot.vrep.simxAuxiliaryConsolePrint(robot.v_clientID,...
               robot.v_console, [], robot.vrep.simx_opmode_oneshot_wait); 
            
        case 'create'
            % Creates a console with 20 lines and will close it at simulation end
            address = java.net.InetAddress.getLocalHost;
            IPaddress = char(address.getHostAddress);
            [result, robot.v_console] = robot.vrep.simxAuxiliaryConsoleOpen ...
                (robot.v_clientID, strcat('Remote Console [', IPaddress, ']'), ...
                20 ,1,[],[],[],[], robot.vrep.simx_opmode_oneshot_wait);
            
        case 'delete'
            % Deletes the auxiliary console
            [result] = robot.vrep.simxAuxiliaryConsoleClose(robot.v_clientID, ...
                robot.v_console, robot.vrep.simx_opmode_oneshot);
            
        case 'show'
            % Shows the auxiliary console
            [result] = robot.vrep.simxAuxiliaryConsoleShow(robot.v_clientID, ...
                robot.v_console, 1, robot.vrep.simx_opmode_oneshot_wait);
            
        case 'hide'
            % Hides the auxiliary console
            [result] = robot.vrep.simxAuxiliaryConsoleShow(robot.v_clientID, ...
                robot.v_console, 0, robot.vrep.simx_opmode_oneshot_wait);
        otherwise
            % Command not recognized
            error = 1;
    end
    
    % Checking for errors
    returnCode = error;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end     
end