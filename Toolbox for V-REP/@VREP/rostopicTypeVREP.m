function [msg_type, returnCode] = rostopicTypeVREP(robot, topic)
%ROSTOPICTYPEVREP gets the message type for a specific topic.
%
%   [msg_type, returnCode] = rostopicTypeVREP(robot, topic)
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

    % Creating the masc for searching
    masc = strcmp(robot.v_topics, topic);
       
    % Searching the  max in the masc
    [maximum, index] = max(masc);
       
    % Cheking errors
    if(maximum == 1)        
        msg_type   = robot.v_topics_types(index);
        returnCode = 0;
    else
        msg_type   = [];
        returnCode = -1;        
    end
end