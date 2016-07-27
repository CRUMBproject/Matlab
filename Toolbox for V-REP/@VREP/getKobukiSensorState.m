function [v_msg, returnCode] = getKobukiSensorState(robot)
%GETKOBUKISENSORSTATE gets the Kobuki's sensor state message, which
%includes many sensors and encoders.
%
%   [v_msg, returnCode] = getKobukiSensorState(robot)
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

% Structure of the message:
%   uint8 BUMPER_RIGHT              = 1
%   uint8 BUMPER_CENTRE             = 2
%   uint8 BUMPER_LEFT               = 4
%   uint8 WHEEL_DROP_RIGHT          = 1
%   uint8 WHEEL_DROP_LEFT           = 2
%   uint8 CLIFF_RIGHT               = 1
%   uint8 CLIFF_CENTRE              = 2
%   uint8 CLIFF_LEFT                = 4
%   uint8 BUTTON0                   = 1
%   uint8 BUTTON1                   = 2
%   uint8 BUTTON2                   = 4
%   uint8 DISCHARGING               = 0
%   uint8 DOCKING_CHARGED           = 2
%   uint8 DOCKING_CHARGING          = 6
%   uint8 ADAPTER_CHARGED           = 18
%   uint8 ADAPTER_CHARGING          = 22
%   uint8 OVER_CURRENT_LEFT_WHEEL   = 1
%   uint8 OVER_CURRENT_RIGHT_WHEEL  = 2
%   uint8 OVER_CURRENT_BOTH_WHEELS  = 3
%   uint8 DIGITAL_INPUT0            = 1
%   uint8 DIGITAL_INPUT1            = 2
%   uint8 DIGITAL_INPUT2            = 4
%   uint8 DIGITAL_INPUT3            = 8
%   uint8 DB25_TEST_BOARD_CONNECTED = 64
%   Header header
%   uint16 time_stamp
%   uint8  bumper
%   uint8  wheel_drop
%   uint8  cliff
%   uint16 left_encoder
%   uint16 right_encoder
%   int8   left_pwm
%   int8   right_pwm
%   uint8  buttons
%   uint8  charger
%   uint8  battery
%   uint16[] bottom 
%   uint8[] current
%   uint8   over_current
%   uint16   digital_input
%   uint16[] analog_input

    % Getting the sensor state data from V-REP
    [result, statesData] = robot.vrep.simxGetStringSignal(robot.v_clientID,...
            'Turtlebot2_WidowX_kobuki_sensor_state', robot.vrep.simx_opmode_buffer);
    
    % Unpacking the sensor states data
    states = robot.vrep.simxUnpackFloats(statesData);
    
    % Creating the message
    v_msg = rosmessage('kobuki_msgs/SensorState');
    
    % Setting the message header
    v_msg.Header = robot.getMessageHeader();

    % Setting the time stamp
    v_msg.TimeStamp = uint16(states(1));
    
    % Setting the bumper's field
    bumper_right  = states(2);
    bumper_center = states(3);
    bumper_left   = states(4);        
    v_msg.Bumper = uint8(bumper_right + 2 * bumper_center + 4 * bumper_left);

    % Setting the wheel drops's field
    wheel_drop_right = states(5);
    wheel_drop_left  = states(6);
    v_msg.WheelDrop = uint8(wheel_drop_right + 2 * wheel_drop_left);

    % Setting the cliff's fields   
    cliff_right       = states(7);
    cliff_center      = states(8);
    cliff_left        = states(9);
    cliff_right_dist  = states(10);
    cliff_center_dist = states(11);
    cliff_left_dist   = states(12);
    v_msg.Cliff  = uint8(cliff_right + 2 * cliff_center + 4 * cliff_left);
    v_msg.Bottom = uint16([cliff_right_dist, cliff_center_dist, cliff_left_dist]);

    % Setting the encoder's fields
    v_msg.RightEncoder = uint16(states(13));
    v_msg.LeftEncoder  = uint16(states(14));

    % Setting the PWM's fields (not used)
    v_msg.LeftPwm       = -1;
    v_msg.RightPwm      = -1;

    % Setting the button's fields
    button_0       = states(15);
    button_1       = states(16);
    button_2       = states(17);
    v_msg.Buttons  = uint8(button_0 + 2 * button_1 + 4 * button_2);

    % Setting the charger's field (not used)
    v_msg.Charger      = -1;

    % Setting the battery's field (not used)
    v_msg.Battery      = -1;

    % Setting the current's fields (not used)
    v_msg.Current      = 0;
    v_msg.OverCurrent  = 0;

	% Setting the input's fields (not used)
    v_msg.DigitalInput = 0;
    v_msg.AnalogInput  = 0;

    % Checking for errors
    returnCode = 0;
    if(result ~= robot.vrep.simx_return_ok)
        returnCode = -1;
    end 
end