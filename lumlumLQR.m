A_=[0 1;0 -1/5];
B_=[0; 1];

%cheap control
Q_=[1 0; 
    0 1];
R_=[0.01];
K_=lqr(A_,B_,Q_,R_)
x0=[pi;-2];
sim('lumlumLQR_simulink_model')

t=out.Sim_Z.time;
x1=out.Sim_Z.signals.values(:,1);
x2=out.Sim_Z.signals.values(:,2);
u1=out.Sim_U.signals.values(:,1);  

figure
subplot(2,1,1)
plot(t,x1,'linewidth', 2)
ylabel('x1')
grid on

subplot(2,1,2)
plot(t,x2,'linewidth', 2)
ylabel('x2')
grid on


figure
plot(t,u1,'linewidth', 2)
ylabel('u1')
grid on