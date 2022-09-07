A= [ 0    1.0000         0         0         0         0         0;
   -4.7059   -0.0880         0         0         0    1.3590         0;
         0         0         0    1.0000         0         0         0;
         0         0         0   -5.0000  -50.0000         0    4.5000;
         0         0         0         0   -0.5000    0.2200         0;
         0         0         0         0         0   -0.9090         0;
         0         0         0         0         0         0   -1.0000;];
     
     B= [ 0         0;
         0         0;
         0         0;
         0         0;
   -0.3500         0;
    1.0000         0;
         0    0.8000;];
     CTL= rank(ctrb(A,B))
C=eye(7);
D=zeros(7,2);
Q=diag([4000 1 1 1 1 1 1]);
R=[1 0;
    0 1];

     
      %C= [1     0     0     0     0     0     0;
      %0     0     1     0     0     0     0;]
%  D= zeros(2)
%  states ={'x1' 'x2' 'x3' 'x4' 'x5' 'x6' 'x7'}
%  inputs ={'u1','u2'}
%  outputs ={'p1','p2'}
%  sysmimo= ss(A,B,C,D, 'statename', states, 'inputname', inputs, 'outputname', outputs)
% J=(tf(sysmimo));

eig(A);
eigen_desired=[-200;-302;-150;-174;-195;-6;-87];
%K1= place(A,B,eigen_desired)
[K,S,E]= lqr(A,B,Q,R);
K
%gain according to NIT Thesis
%K2=   [22.7462    1.5716    -7.2280    3.9397   -52.6533    0.1295    3.6697;
 %   2.5257    0.0494    -1.0458    0.6041   -3.3438    -0.6577    0.1323]

