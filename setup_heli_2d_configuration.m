% SETUP_HELI_2D_CONFIGURATION

%

% SETUP_HELI_2D_CONFIGURATION sets and returns the model model parameters 

% of the Quanser 2 DOF Helicopter plant.

%

%

% Copyright (C) 2006 Quanser Consulting Inc.

% Quanser Consulting Inc.

%

%

function [ K_pp, K_yy, K_yp, K_py, J_eq_p, J_eq_y, B_eq_p, B_eq_y, m_heli, l_cm, g ] = setup_heli_2d_configuration( )

%

% Gravitational Constant (m/s^2)

g = 9.81;

% Pitch and Yaw Motor Armature Resistance (Ohm)

R_m_p = 0.83;

R_m_y = 1.60;

% Pitch and Yaw Motor Current-Torque Constant (N.m/A)

K_t_p = 0.0182;

K_t_y = 0.0109;

% Pitch and Yaw Motor Voltage-Torque Constant (N.m/V)

K_yp = K_t_p / R_m_p;

K_py = K_t_y / R_m_y;

% Pitch and Yaw Viscous Damping Constant (N.m.s/rad)

B_eq_p = 0.8;  % tuned while running simulation and experiment in parallel

B_eq_y = 0.318; % identified as described in manual 

% Mass of the Helicopter (kg)

m_heli = 1.3872;

% Mass of the Helicopter with Yoke (kg)

m_heli_case = 1.421;

% Distance from Pitch Pivot to Pitch/Front Motor and Yaw/Back Motor(m)

r_p = 7.75*0.0254;

r_y = (6+5/8)*0.0254;

% Pitch and Yaw Propeller Force-Thrust Constant found Experimentally (N/V)

K_f_p = 5*0.2074;

K_f_y = 3*0.1426;

% Pitch and Yaw Propeller Torque-Thrust Constant found Experimentally (N.m/V)

K_pp = K_f_p * r_p;

K_yy = K_f_y * r_y;

% Mass of Pitch/Front Motor and Yaw/Back Motor (kg)

m_motor_p = 0.292;

m_motor_y = 0.128;

% Mass of Pitch/Front and Yaw/Back Guard + Propeller(kg)

m_shield = 0.143 + 0.024;

% Total Mass of Pitch and Yaw Motor/Shield Assemblies (kg)

m_props = ( m_motor_p + m_motor_y + 2 * m_shield );

% Mass of Helicopter Body moving about Pitch Axis (kg)

m_body_p = m_heli - m_props; 

% Mass of Helicopter Body moving about Yaw Axis (kg)

m_body_y = m_heli_case - m_props; 

% Mass of Metal Shaft Moving about Yaw Axis (kg)

m_shaft = 0.151;

% Total Length of Helicopter Body (m)

L_body = 19*0.0254;

% Helicopter Center of Mass from Pivot along Pitch Axis (m)

l_cm = ( (m_motor_p + m_shield ) * r_p +  (m_motor_y + m_shield ) * r_y ) / ( m_props ) ;

% Length of Metal Shaft Moving about Yaw Axis through slip ring (m)

L_shaft = 11 * 0.0254;

% Pitch and Yaw Motor Rotor Moment of Inertia (kg.m^2)

J_m_p = 1.91e-6; %1.7070e-5;

J_m_y = 1.374e-4; %1.4471e-5;

% Moment of Inertia of Helicopter Body about its CM (kg.m^2)

J_body_p = m_body_p * L_body^2 / 12;

J_body_y = m_body_y * L_body^2 / 12;

% Moment of Inertia of Metal Shaft about Yaw Axis (kg.m^2)

J_shaft = m_shaft * L_shaft^2 / 3; %;0.0851;

% Moment of Inertia of Pitch Motor + Guard Assembly about Pivot (kg.m^2)

J_p = ( m_motor_p + m_shield ) * r_p^2;

% Moment of Inertia of Yaw Motor + Guard Assembly about Pivot (kg.m^2)

J_y = ( m_motor_y + m_shield ) * r_y^2;

% Equivalent Moment of Inertia about Pitch and Yaw Axis (kg.m^2)

J_eq_p = J_m_p + J_body_p + J_p + J_y;

J_eq_y = J_m_y + J_body_y + J_p + J_y + J_shaft;

%

% end of setup_heli_2d_configuration()