A1=[0 0 1 0 0 0 0;
    0 0 0 1 0 0 0;
    -4.34 0 -0.0882 0 1.24 0 0;
0 0 0 -5 1.4823 3.6 18.75;
0 0 0 0 -0.8333 0 0;
0 0 0 0 0 -1 0;
0 0 0 0 -0.0169 0 -0.5]

B1=[0 0;
    0 0;
    0 0;
    0 0;
    1 0;
    0 1;
    0 0;]
C1= eye(7);
D1= zeros(7,2);
Q1=diag([5000 5000 5000 1 1 1 1]);
R1=[0.1 0;
    0 100];
[K1,S1,E1]= lqr(A1,B1,Q1,R1);
K1