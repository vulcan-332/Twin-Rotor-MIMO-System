%Christopher Lum
%lum@uw.edu
%
%Investigate practical implementation issues with full state feedback
%control.
%
%This file is designed to accompany the Yansube video at
%https://youtu.be/9vCTokJ5RQ8

%Version History
%11/28/18: Created from lecture notes to accompany YouTube

clear
clc
close all

%% Plant Model (DC Motor)
%Define constants
R   = 0.05;         %resistance of measurement resistor (ohms)
KV  = 0.09854;      %electrical machine constant (volt sec)
KT  = 0.09854;      %mechanical machine constant (Nm/amp)
Rm  = 1.5398;       %resistance of motor (ohms)
La  = 0.0015581;    %inductance of motor (henries)
c   = 0.00039719;   %coefficient of viscous friction (Nm sec)
Jm  = 0.00137;      %moment of inertia about axis of rotation (kg*m^2)

%State space representation
A = [0 1 0;
     0 -c/Jm KT/Jm;
     0 -KV/La -(R+Rm)/La];
 
B = [0 0;
    0 -1/Jm;
    1/La 0];

C = eye(3);

D = zeros(3,2);

%Look at open loop eigenvalues of system
open_loop_poles = eig(A)

%% Design Full State Feedback Controller
%Verify that the system is controllable
Pc = ctrb(A,B(:,1))
rank(Pc)

%Choose desired closed loop pole locations (uncomment desired behavior)
desired_closed_loop_poles = [-100; -110; -120];                             %aggressive controller
% desired_closed_loop_poles = [-5; -30; -400];                                %less aggressive controller
% desired_closed_loop_poles = [-2; open_loop_poles(2); open_loop_poles(3)];   %only move poles associated with states we directly measure

%Compute full state feedback gain
K = place(A,B(:,1),desired_closed_loop_poles)

%% Simulate System
t_final = 2;
x0 = [72*pi/180;
    2*pi;
    -1];

sim('lumlummotor')

%From model with full state feedback
t       = ans.Sim_X.time;
x1      = ans.Sim_X.signals.values(:,1);
x2      = ans.Sim_X.signals.values(:,2);
x3      = ans.Sim_X.signals.values(:,3);
Va      = ans.Sim_Va.signals.values(:,1);

% x1_tilde = ans.Sim_X_tilde.signals.values(:,1);
% x2_tilde = ans.Sim_X_tilde.signals.values(:,2);
% x3_tilde = ans.Sim_X_tilde.signals.values(:,3);
% Va_tilde = ans.Sim_Va_tilde.signals.values(:,1);

%Create plots
figure
subplot(3,1,1)
plot(t, x1, 'LineWidth', 2)
ylabel('x_1(t)')
grid on
title('Response of System with Full State Feedback Controller')

subplot(3,1,2)
plot(t, x2, 'LineWidth', 2)
ylabel('x_2(t)')
grid on

subplot(3,1,3)
plot(t, x3,'LineWidth', 2)
ylabel('x_3(t)')
grid on
xlabel('Time (sec)')

figure
plot(t, Va, 'LineWidth', 2)
ylabel('V_a(t)')
grid on
title('Control Signal, V_a')