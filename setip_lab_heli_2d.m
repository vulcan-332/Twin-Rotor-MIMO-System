% SETUP_LAB_HELI_2D

%

% 2 DOF Helicopter (2DHELI) Control Lab: 

% Design of a FF+LQR+I position controller

% 

% SETUP_LAB_HELI_2D sets the model parameters and set the controller

% parameters for the Quanser 2DOF Helicopter system.

%

% Copyright (C) 2005 Quanser Consulting Inc.

% Quanser Consulting Inc.

%

clear all;

%

% ############### USER-DEFINED 2DOF HELI CONFIGURATION ###############

% Cable Gain used for yaw and pitch axes.

K_CABLE_P = 5;

K_CABLE_Y = 3;

% UPM Maximum Output Voltage (V): YAW has UPM-15-03 and PITCH has UPM-24-05

VMAX_UPM_P = 24;

VMAX_UPM_Y = 15;

% Digital-to-Analog Maximum Voltage (V): set to 10 for Q4/Q8 cards

VMAX_DAC = 10;

% Pitch and Yaw Axis Encoder Resolution (rad/count)

K_EC_P = - 2 * pi / ( 4 * 1024 );

K_EC_Y = 2 * pi / ( 8 * 1024 );

% Initial Angle of Pitch (rad)

theta_0 = 0;

%

% ############### END OF USER-DEFINED DOF HELI CONFIGURATION ###############

%

%

% ############### USER-DEFINED CONTROLLER/FILTER DESIGN ###############

% Anti-windup: integrator saturation (V)

SAT_INT_ERR_PITCH = 5;

SAT_INT_ERR_YAW = 5;

% Anti-windup: integrator reset time (s)

Tr_p = 1;

Tr_y = 1;

%

% Type of Controller: set it to 'LQR_AUTO' or 'MANUAL'  

CONTROLLER_TYPE = 'LQR_AUTO';    % LQR controller design: automatic mode

%CONTROLLER_TYPE = 'MANUAL';    % controller design: manual mode

%

% Specifications of a second-order low-pass filter

wcf = 2 * pi * 20; % filter cutting frequency

zetaf = 0.85;        % filter damping ratio

% ############### END OF USER-DEFINED CONTROLLER DESIGN ###############

%

% ############### USER-DEFINED JOYSTICK SETTINGS ###############

% Joystick input X sensitivity used for yaw (deg/s/V)

K_JOYSTICK_X = 85;

% Joystick input Y sensitivity used for pitch (deg/s/V)

K_JOYSTICK_Y = 85;

% Joystick input X sensitivity used for yaw (V/s/V)

K_JOYSTICK_V_X = 10;

% Joystick input Y sensitivity used for pitch (V/s/V)

K_JOYSTICK_V_Y = 5;

% Pitch integrator saturation of joystick (deg)

INT_JOYSTICK_SAT_LOWER = theta_0 * 180 / pi;

INT_JOYSTICK_SAT_UPPER = abs(theta_0)  * 180 / pi;

% Deadzone of joystick: set input ranging from -DZ to +DZ to 0 (V)

JOYSTICK_X_DZ = 0.25;

JOYSTICK_Y_DZ = 0.25;

% ############### END OF USER-DEFINED JOYSTICK SETTINGS ###############

%

% Set the model parameters of the 2DOF HELI.

% These parameters are used for model representation and controller design.

[ K_pp, K_yy, K_yp, K_py, J_eq_p, J_eq_y, B_p, B_y, m_heli, l_cm, g] = setup_heli_2d_configuration();

%

% For the following state vector: X = [ theta; psi; theta_dot; psi_dot]

% Initialization the state-Space representation of the open-loop System

HELI_2D_ABCD_eqns;

%

if strcmp ( CONTROLLER_TYPE, 'LQR_AUTO' )

    % Feed-forward gain adjustment (V/V)

    K_ff = 1;

    % LQR Controller Design Specifications

    % Q = diag([1 1 0.5 0.5]);

    % R = 0.005*eye(2,2);

    Q = diag([200 200 100 100]);

    R = eye(2,2);

    % Automatically calculate the LQR controller gain

    [ K ] = d_heli_2d_lqr( A, B, C, D, Q, R );    

    % Display the calculated gains

%     disp( ' ' )

%     disp( 'Calculated LQR controller gain elements: ' )

%     disp( [ 'K = [' num2str( K(1,1),3 ) ' V/rad  '  num2str( K(1,2),3 ) ' V/rad  ' num2str( K(1,3),3 ) ' V.s/rad  '  num2str( K(1,4),3 ) ' V.s/rad]'] )

%     disp( [ '    [' num2str( K(2,1),3 ) ' V/rad  '  num2str( K(2,2),3 ) ' V/rad  ' num2str( K(2,3),3 ) ' V.s/rad  '  num2str( K(2,4),3 ) ' V.s/rad]'] )

    %

    % LQR+I Controller Design Specifications

    Qi = diag([Q(1,1) 0.75*Q(2,2) Q(3,3) 2*Q(4,4) 50 50 ]); 

    % Automatically calculate the LQR controller gain

    [ Ki ] = d_heli_2d_lqr_i( A, B, C, D, Qi, R );    

    % Display the calculated gains

    disp( ' ' )

%     disp( 'Calculated LQR+I controller gain elements: ' )

%     disp( [ 'Ki = [' num2str( Ki(1,1),3 ) ' V/rad  '  num2str( Ki(1,2),3 ) ' V/rad  ' num2str( Ki(1,3),3 ) ' V.s/rad  '  num2str( Ki(1,4),3 ) ' V.s/rad ' num2str( Ki(1,5),3 ) ' V/(rad.s) ' num2str( Ki(1,6),3 ) ' V/(rad.s)]'] )

%     disp( [ '    [' num2str( Ki(2,1),3 ) ' V/rad  '  num2str( Ki(2,2),3 ) ' V/rad  ' num2str( Ki(2,3),3 ) ' V.s/rad  '  num2str( Ki(2,4),3 ) ' V.s/rad ' num2str( Ki(2,5),3 ) ' V/(rad.s) ' num2str( Ki(2,6),3 ) ' V/(rad.s)]' ] )    

elseif strcmp ( CONTROLLER_TYPE, 'MANUAL' )

%     disp( [ 'K = [' 0 ' V/rad  '  0 ' V/rad  ' 0 ' V.s/rad  '  0 ' V.s/rad]'] )

%     disp( ' ' )

%     disp( 'STATUS: manual mode' ) 

%     disp( 'The model parameters of your Single Pendulum and IP01 or IP02 system have been set.' )

%     disp( 'You can now design your state-feedback position controller.' )

%     disp( ' ' )

else

    error( 'Error: Please set the type of controller that you wish to implement.' )

end
