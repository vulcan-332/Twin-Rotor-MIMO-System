% A=[0 3;2 4]
% B=[-2 1;1 1]
% rank(ctrb(A,B))        
% p_desired = [-5+2*i -5-2*i];
% K= place(A,B,p_desired)
% ACL= A-B*K;
% Eigenvalues = eig(ACL);

%practical implimentations video
%DC MOTOR
A= [0 1 0;
    0 -0.29 71.93;
    0 -63.24 -1020.35;]
B= [0 0;0 -729.92; 641.81 0;]
CTR=rank(ctrb(A,B))
eig(A)
%desired_eigen= [-100; -110; -120]
desired_eigen= [-5; -4.8; -1015.9]
K= place(A,B,desired_eigen) 
C= eye(3);
D= zeros(3,2);
sim('lumlummotor')
% t= ans.Sim_X.time;
% x1=ans.Sim_X.signals.values(:,1);
% x2=ans.Sim_X.signals.values(:,2);
% x3=ans.Sim_X.signals.values(:,3);
% Va=ans.Sim_Va.signals.values(:,1);

figure
subplot(3,1,1)
plot(t,x1,'Linewidth',2)
ylabel('x1(t)')
grid on

subplot(3,1,2)
plot(t,x2,'Linewidth',2)
ylabel('x2(t)')
grid on

subplot(3,1,3)
plot(t,x3,'Linewidth',2)
ylabel('x3(t)')
grid on

figure
plot(t, Va, 'linewidth', 2)
ylabel('Va(t)')
grid on