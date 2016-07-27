function [enabled] = isVREPEnabled(robot)
%ISVREPENABLED returns if V-REP mode is enabled or not
%  
% [enabled] = isVREPEnabled(robot)
%
% 'enabled' will be set to 1 if current operating mode is V-REP or Mixed,
% otherwise it will be set to 0.

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

    if(robot.operating_mode == 0 || robot.operating_mode == 2)
        enabled = 1;       
    else
        enabled = 0;
    end
end