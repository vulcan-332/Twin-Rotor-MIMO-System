% D_HELI_2D_LQR

%

% Control Lab: Design of a LQR Controller for the Quanser 2 DOF Helicopter.

%

% D_HELI_2D_LQR designs a LQR controller for the 2D HELI system,

% and returns the corresponding gain vector: K

%

% Copyright (C) 2006 Quanser Consulting Inc.

% Quanser Consulting Inc.





function [ K ] = d_heli_2d_lqr( A, B, C, D, Q, R )

%PLOT_RESPONSE = 'YES';

PLOT_RESPONSE = 'NO';

%SYS_ANALYSIS = 'YES';

SYS_ANALYSIS = 'NO';

%

% Open Loop system

HELI_2D_OL_SYS = ss( A, B, C, D, 'statename', { '\theta' '\psi' '\theta_dot' '\psi_dot' }, 'inputname', {'u_p' 'u_y'} , 'outputname', { '\theta' '\psi' '\theta_dot' '\psi_dot' } );

%

% calculate the LQR gain vector, K

[ K, S, EIG_CL ] = lqr( A, B, Q, R );

%

% Closed-Loop State-Space Model

A_CL = A - B * K;

B_CL = B;

C_CL = C;

D_CL = D;

%

% Closed-Loop System

HELI_2D_CL_SYS = ss( A_CL, B_CL, C_CL, D_CL, 'statename', { '\theta' '\psi' '\theta_dot' '\psi_dot' }, 'inputname', {'\theta_d' '\psi_d'} , 'outputname', { '\theta' '\psi' '\theta_dot' '\psi_dot' } );

%

if strcmp( PLOT_RESPONSE, 'YES' )

    % initialization

    close all

    fig_h = 1; % figure handle number

    %

    % let's look at the step response of the closed-loop system

    % unit step closed-loop response of all 4 states

    figure( fig_h ) 

    % plotting of a step response

    step( HELI_2D_CL_SYS )

    set( fig_h, 'name', strcat( 'Closed-Loop System: 2 DOF HELI + LQR' ) )

    grid on

    orient tall

    %

    %

    % unit step closed-loop response

    [ yss, tss, xss ] = step( HELI_2D_CL_SYS );

    fig_h = fig_h + 1;

    figure( fig_h )

    subplot( 2, 1, 1 )

    plot( tss, ( yss( :, 1, 1 ) * 180 / pi ) )

    grid on

    title( 'Unit Step Response on \theta' )

    xlabel( 'Time (s)' )

    ylabel( '\theta (deg)' )

    subplot( 2, 1, 2 )

    plot( tss, ( yss( :, 2, 1 ) * 180 / pi ) )

    grid on

    xlabel( 'Time (s)' )

    ylabel( '\psi (deg)' )

    set( fig_h, 'name', strcat( 'Closed-Loop System: 2 DOF HELI + LQR' ) )

    %

    fig_h = fig_h + 1;

    figure( fig_h )

    subplot( 2, 1, 1 )

    plot( tss, ( yss( :, 1, 2 ) * 180 / pi ) )

    grid on

    title( 'Unit Step Response on \psi' )

    xlabel( 'Time (s)' )

    ylabel( '\theta (deg)' )

    subplot( 2, 1, 2 )

    plot( tss, ( yss( :, 2, 2 ) * 180 / pi ) )

    grid on

    xlabel( 'Time (s)' )

    ylabel( '\psi (deg)' )

    set( fig_h, 'name', strcat( 'Closed-Loop System: 2 DOF HELI + LQR' ) )

    fig_h = fig_h + 1;

end



% carry out some additional system analysis

if strcmp( SYS_ANALYSIS, 'YES' )    

    ULABELS = [ 'u_p u_y' ];

    XLABELS = [ '\theta \psi \theta_dot \psi_dot' ];

    YLABELS = XLABELS;

    % print the Open-Loop State-Space Matrices

    disp( 'Open-Loop System' )

    printsys( A, B, C, D, ULABELS, YLABELS, XLABELS )

    % one unstable open-loop pole

    OL_poles = eig( A )

    % print the Closed-Loop State-Space Matrices

    disp( 'Closed-Loop System' )

    printsys( A_CL, B_CL, C_CL, D_CL, ULABELS, YLABELS, XLABELS )

    % Closed-Loop poles, damping, and natural frequency

    damp( HELI_2D_CL_SYS )

    % or: Closed-Loop eigenvalues

    CL_poles = eig( A_CL ); % = EIG_CL

end

% end of function 'd_heli_2d_lqr( )'
