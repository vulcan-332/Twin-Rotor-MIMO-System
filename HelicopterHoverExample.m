%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	HelicopterHoverExample.m
%		This script implements the Helicopter Hover Example from
%			http://wikis.controltheorypro.com/index.php?title=Helicopter_Hover_Example
%
%		The webpage is an example of modeling the flight dynamics of a Blackhawk
%			helicopter linearized around the hovering point.  Near hover the flight
%			dynamics for pitch attitude and horizontal speed can be decoupled.
%
%		The Blackhawk Helicopter 3 DOF model details are
%		from pp. 5-10 of the thesis:
%
%		Lim, Chen-I '"DEVELOPMENT OF INTERACTIVE MODELING, SIMULATION, ANIMATION, 
%		AND REAL-TIME CONTROL (MoSART) TOOLS FOR RESEARCH AND EDUCATION." 
%		Master's Thesis, Arizona State University, Tempe, AZ December 1999.
%
%	---	Arguments, N/A this is a script
%
% Sub-programs (end of file):
%   N/A
%
% Called Routines (non-Matlab):
%   N/A
%
% Structures:
%   N/A
%
% Notes:
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	Copyright, Gabe Spradlin, 2008 --- http://wikis.ControlTheoryPro.com
%		All rights reserved
%		You may use this function freely for any use except redistribution.
%			In other words this function is open to everyone for personal,
%			educational, and commercial uses.  However, if you wish to provide
%			end users with this function please do so by providing the following
%			link to it:
%
%			http://wikis.controltheorypro.com/index.php?title=Image:HelicopterHoverExample.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Revision History:
%
% Original
% 05/16/08
% G Spradlin
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%	---	Dynamic Parameters detailed on pg. 6 of the thesis and Table 1 of
%				http://wikis.controltheorypro.com/index.php?title=Helicopter_Hover_Example
Z_theta 								= 5.95;				%	{ft} / {deg s^2}
Z_w											= -0.346;			%	{1} / {s}
g												= 32.283;			%	{ft} / {s^2}
X_u											= -0.06;			%	{ft/s^2} / {1/s}
M_q											= -3.1;				%	{deg/s^2} / {deg/s} = {1}/{s}
X_Blc										= 0.478;			%	{ft/s^2}/{deg}
M_u											= 2.3493;			%	{deg/s^2}/{ft/s}
M_Blc										= -47.24;			%	{deg/s^2}/{deg} = {1}/{s^2}

%	--- Some parameters are provided with units of degrees; Convert deg to rad
%				use in the transfer functions
Z_theta									= Z_theta * (180/pi);		%	{ft}/{deg s^2} * {deg}/{rad} = {ft}/{rad s^2}
X_Blc										= X_Blc * (180/pi);			%	{ft/s^2}/{deg} * {deg}/{rad} = {ft/s^2}/{rad}
M_u											= M_u * (pi/180);				%	{deg/s^2}/{ft/s} = {deg}/{ft*s} * {rad/deg} = {rad}/{ft*s}


% ---	Plant transfer functions
%				Vertical Dynamics
z_over_Theta_c					= ss(tf([Z_theta], [1, Z_w 0]));

%				Horizontal Speed Dynamics
xdot_over_B_lc					= ss(X_Blc * ...																	%	Gain up front
														 tf([1, -M_q, -(g*M_Blc)/X_Blc], ...					%	TF numerator
														    [1, -(X_u + M_q), +M_q*X_u, +g*M_u]));		%	TF denominator

%				Pitch Attitude Dynamics
Theta_over_B_lc					= ss(M_Blc * ...
														 tf([1, ((X_Blc*M_u/M_Blc) - X_u)], ...
														    [1, -(X_u + M_q), M_q*X_u, +g*M_u]));


%	---	The thesis includes a block where a sensor dynamics should be.
%				The transfer function provided looks a lot like a filtered sensor.  The bandwidth of
%				the sensor would be 50/(2*pi).
wn											= 50;												%	Units were not provided for this but the MATLAB default is rad/s not Hz
z												= 0;												
k												= 1;
b												= 2;												%	Units were not provided for this but the MATLAB default is rad/s not Hz
pitch_sensor						= ss(zpk([], [-wn -wn], [wn^2])) * ss(zpk([-b -b], [], 1/b^2));
b												= 1;												%	Units were not provided for this but the MATLAB default is rad/s not Hz
speed_sensor						= ss(zpk([], [-wn -wn], [wn^2])) * ss(zpk([-b -b], [], 1/b^2));

%	Controllers
%	---	Thesis Controllers
%				These are the controllers from the thesis.  The thesis states that these controllers produce good
%				performance.  However, they were not stable in my Simulink model.
k_v											= 1;
k_theta									= -1;
a												= 2.5;
k_h											= 0.0005;

Theta_c									= k_v;
Pitch_B_lc							= ss(zpk([], [0], [k_theta]));
Speed_B_lc							= ss(zpk([-a], [0], [k_h]));

%	---	Set the damping and natural frequency of the unstable plant
z												= 0.1082;
wn											= 0.6402;

%	---	My controller designs
tp											= 0.5;
zd											= 1/sqrt(2);
wd											= sqrt(2)*(pi/tp);
a												= 10*wd;

%	Assume controller form of
%		K = tf([B1 B0], [A1 A0]);

%	Desired characteristic equation
%		phi = (s + a) * (s^2 + 2*zd*wd*s + wd^2)

%	---	From the Diaphantine eqn
A1											= 1;
A0											= 2*zd*wd + a - 2*z*wn;
B0											= (a*wd^2 - A0*wn^2)/wn^2;
B1											= (wd^2 + 2*zd*wd*a - 2*z*wn*A0 - A1*wn^2)/wn^2;

Knum										= [B1 B0];
Kden										= [A1 A0];
K 											= tf([B1 B0], [A1 A0]);

%	Run the pitch attitude Sim --- blackhawk_3dof.mdl
%	---	Create pitch plant numerator and denominator
[plant_num, plant_den] 	= tfdata(Theta_over_B_lc, 'v');

%	---	Run blackhawk simulation
sim('blackhawk_3dof')


%	Run the feedforward Sim --- ff_blackhawk_3dof.mdl
%	---	Create the Honeywell GG5300 Sensor
z_GG										= 1/sqrt(2);
w_GG										= 100 * (2*pi);
ff_blackhawk_rate_seed	= 23300;
ff_blackhawk_sensor_seed= 50000;
Int_Sample_Time					= 1/1000;

%	---	Run feedforward blackhawk sim
sim('ff_blackhawk_3dof')


%	---	Post Process the simulation data
%		Available Scope Data
%				Pitch:					Pitch Command Following
%				ff_Err:					Feedforward Position Error
%				ff_Pitch:				Feedforward Command Following

%				Pitch
%					1 signal port, 2 signals --- 1st: Reference, 2nd: Command Following
h(1)										= figure('Name', 'Pitch');
plot(Pitch.time, Pitch.signals(1).values(:, 1), 'b', Pitch.time, Pitch.signals(1).values(:, 2), 'r-.')
grid on
ylabel('Pitch Angle (rad)')
xlabel('Time (sec)')
title({'Reference Command Following', 'blackhawk_3dof.mdl'}, 'Interpreter', 'None')
legend('Command', 'True Position', 'Location', 'SouthEast')

h(end+1)								= figure('Name', 'Pitch, Zoom');
plot(Pitch.time, Pitch.signals(1).values(:, 1), 'b', Pitch.time, Pitch.signals(1).values(:, 2), 'r-.')
grid on
ylabel('Pitch Angle (rad)')
xlabel('Time (sec)')
title({'Reference Command Following', 'blackhawk_3dof.mdl'}, 'Interpreter', 'None')
legend('Command', 'True Position', 'Location', 'SouthEast')
set(gca, 'ylim', [0.98 1.02])

%				ff_Pitch
%					1 signal port, 2 signals --- 1st: Reference or desired angle, 2nd: Current True Angle
h(end+1)								= figure('Name', 'Feedforward Pitch');
plot(ff_Pitch.time, ff_Pitch.signals(1).values(:, 1), 'b', ff_Pitch.time, ff_Pitch.signals(1).values(:, 2), 'r-.')
grid on
ylabel('Pitch Angle (rad)')
xlabel('Time (sec)')
title({'Feedforward Reference Command Following', 'ff_blackhawk_3dof.mdl'}, 'Interpreter', 'None')
legend('Command', 'True Position', 'Location', 'SouthEast')

h(end+1)								= figure('Name', 'Feedforward Pitch, Zoom');
plot(ff_Pitch.time, ff_Pitch.signals(1).values(:, 1), 'b', ff_Pitch.time, ff_Pitch.signals(1).values(:, 2), 'r-.')
grid on
ylabel('Pitch Angle (rad)')
xlabel('Time (sec)')
title({'Feedforward Reference Command Following', 'ff_blackhawk_3dof.mdl'}, 'Interpreter', 'None')
legend('Command', 'True Position', 'Location', 'SouthEast')
set(gca, 'ylim', [0.0990 0.1008])

%				ff_Err
%					1 signal port, 1 signal --- Difference between commanded and true angle
h(end+1)								= figure('Name', 'Feedforward Error');
plot(ff_Err.time, ff_Err.signals(1).values(:, 1), 'b')
grid on
ylabel('Pitch Angle Error (rad)')
xlabel('Time (sec)')
title({'Feedforward Position Error', 'ff_blackhawk_3dof.mdl'}, 'Interpreter', 'None')

h(end+1)								= figure('Name', 'Feedforward Error, Zoom');
plot(ff_Err.time, ff_Err.signals(1).values(:, 1), 'b')
grid on
ylabel('Pitch Angle Error (rad)')
xlabel('Time (sec)')
title({'Feedforward Position Error', 'ff_blackhawk_3dof.mdl'}, 'Interpreter', 'None')
set(gca, 'ylim', [-1 1]*1E-3)
