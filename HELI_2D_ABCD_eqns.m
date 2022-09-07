% Matlab equation file: "HELI_2D_ABCD_eqns.m"

% Open-Loop State-Space Matrices: A, B, C, and D

% for the Quanser 2 DOF Helicopter Experiment.



A( 1, 1 ) = 0;

A( 1, 2 ) = 0;

A( 1, 3 ) = 1;

A( 1, 4 ) = 0;

A( 2, 1 ) = 0;

A( 2, 2 ) = 0;

A( 2, 3 ) = 0;

A( 2, 4 ) = 1;

A( 3, 1 ) = 0;

A( 3, 2 ) = 0;

A( 3, 3 ) = -B_p/(J_eq_p+m_heli*l_cm^2);

A( 3, 4 ) = 0;

A( 4, 1 ) = 0;

A( 4, 2 ) = 0;

A( 4, 3 ) = 0;

A( 4, 4 ) = -B_y/(J_eq_y+m_heli*l_cm^2);



B( 1, 1 ) = 0;

B( 1, 2 ) = 0;

B( 2, 1 ) = 0;

B( 2, 2 ) = 0;

B( 3, 1 ) = K_pp/(J_eq_p+m_heli*l_cm^2);

B( 3, 2 ) = K_py/(J_eq_p+m_heli*l_cm^2);

B( 4, 1 ) = K_yp/(J_eq_y+m_heli*l_cm^2);

B( 4, 2 ) = K_yy/(J_eq_y+m_heli*l_cm^2);



C( 1, 1 ) = 1;

C( 1, 2 ) = 0;

C( 1, 3 ) = 0;

C( 1, 4 ) = 0;

C( 2, 1 ) = 0;

C( 2, 2 ) = 1;

C( 2, 3 ) = 0;

C( 2, 4 ) = 0;

C( 3, 1 ) = 0;

C( 3, 2 ) = 0;

C( 3, 3 ) = 1;

C( 3, 4 ) = 0;

C( 4, 1 ) = 0;

C( 4, 2 ) = 0;

C( 4, 3 ) = 0;

C( 4, 4 ) = 1;



D( 1, 1 ) = 0;

D( 1, 2 ) = 0;

D( 2, 1 ) = 0;

D( 2, 2 ) = 0;

D( 3, 1 ) = 0;

D( 3, 2 ) = 0;

D( 4, 1 ) = 0;

D( 4, 2 ) = 0;
