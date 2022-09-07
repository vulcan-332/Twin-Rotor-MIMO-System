%% Twin Rotor MIMO sytem (TRMS) model. (V.092010), CPOH 2010
% Use this script to load the physical parameters required to excecute the linear and
% non linear models.
%%
%% CPOH 2010
% Heuristic Optimization and Predictive Control Research Group
% Instituto Universitario de Automática e Informática Industrial
% Universidad Politécnica de Valencia
% Valencia, España
%%
%% Authors
% Sergio García-Nieto    - project coordinator -     (sergarro@isa.upv.es)
% Gilberto Reynoso-Meza  - simulink implementation - (gilreyme@posgrado.upv.es)
% Jesús Carrillo Ahumada - parameter identification -
% Christoph Gabriel      - parameter identification -
% Please Cite: "Controller Tuning by means of Multiobjective Optimization Algorithms: a Global Tuning Framework"
%%
%% 
disp('Twin Rotor MIMO system (TRMS) V.092010')
disp('Heuristic Optimization and Predictive Control Research Group')
%%
%% Linear Model
ss.A=zeros(6,6);
ss.A(1,2) = 1.0;
ss.A(2,1) = -7.1373;
ss.A(2,2) = -0.0315;
ss.A(2,3) = 9.4202;
ss.A(3,3) = -0.7519;
ss.A(4,5) = 1.0;
ss.A(5,4) = -0.22995;
ss.A(5,5) = -0.34759;
ss.A(5,6) = 2.50;
ss.A(6,6) = -2.3256;

ss.B=zeros(6,2);
ss.B(2,2) = 0.3543;
ss.B(3,1) = 0.7519;
ss.B(5,1) = 0.7166;
ss.B(6,2) = 2.3256;

ss.C=zeros(2,6);
ss.C(1,1) = 1.0;
ss.C(2,4) = 1.0;

ss.D=zeros(2,2);
%%
%% Observer 

Obs.L=zeros(2,6);
Obs.L(1,1)=0.5010;
Obs.L(1,2)=3.6375;
Obs.L(1,3)=0.8752;
Obs.L(1,4)=0.0179;
Obs.L(1,5)=0.2040;
Obs.L(1,6)=0.0724;
Obs.L(2,1)=0.1097;
Obs.L(2,2)=1.7060;
Obs.L(2,3)=0.6259;
Obs.L(2,4)=0.4031;
Obs.L(2,5)=1.9671;
Obs.L(2,6)=0.1708;

Obs.Ahat=ss.A;
Obs.Bhat=[ss.B,Obs.L'];

Obs.Chat=[1 0 0 0 0 0;
      0 1 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 1 0 0;
      0 0 0 0 1 0;
      0 0 0 0 0 1];


Obs.Dhat=zeros(6,4);
%%
%% State-space feedback controller
% (Your controller goes here)

%K=[-0.265 2.103 3.374 0.996 0.559 0.001 -1.239 -0.497;
%   0.213 0.649 -0.118 3.159 2.418 2.561 0.250 -1.301]; %GK21 in the paper

K=[2.792 3.427 9.619 1.646 2.522 0.377 -3.846 -0.536;
   -0.872 0.230 -4.464 2.275 5.107 3.273 1.313 -0.859]; %GK22in the paper

%K=[0.315 0.556 0.715 0.303 0.228 0.124 -0.376 -0.086;
%   -0.069 0.384 0.254 0.888 0.806 0.840 -0.046 -0.282]; %GK23 in the paper

%K=[0.084 0.541 0.973 0.166 0.402 -0.536 -0.204 -0.022;
%   -0.148 0.334 0.207 0.609 0.655 0.429 0.113 -0.153]; %GK24 in the paper


%%
%% Physical Parameter.
TRMS.g = 9.81;         %
TRMS.lm = 0.228;       %   
TRMS.lt = 0.282;       %
TRMS.kv = 0.0008;      %
TRMS.Tm = 1.33;        %
TRMS.Tt = 0.43;        %
TRMS.kh = 0.0065;      %
TRMS.kc = 0.0043;      %
TRMS.kct = 0.009;      %       
TRMS.kcm = 0.0134;     %        
TRMS.kfhp = 1.508E-07; % 
TRMS.kfvp = 1.71E-06;  %     
TRMS.rms = 0.155;      %
TRMS.rts = 0.10;       %
TRMS.mtr = 0.11;       %  
TRMS.mts = 0.09;       %
TRMS.mt = 0.0150;      %
TRMS.mmr = 0.1255;     %    
TRMS.mcb = 0.068;      %
TRMS.mm = 0.0140;      %    
TRMS.mb = 0.022;       %
TRMS.mms = 0.15;       %     
TRMS.lb = 0.260;       %  
TRMS.lcb = 0.230;      %
TRMS.D = 0.01;         %
TRMS.F = 0.0087;       %
TRMS.Jv = 0.0254;      %       

TRMS.A=(TRMS.mt/2+TRMS.mtr+TRMS.mts)*TRMS.lt;
TRMS.B=(TRMS.mm/2+TRMS.mmr+TRMS.mms)*TRMS.lm;
TRMS.C=(TRMS.mb*TRMS.lb/2)+TRMS.mcb*TRMS.lcb;
TRMS.E=(TRMS.mb*TRMS.lb*TRMS.lb/3+TRMS.mcb)*TRMS.lcb*TRMS.lcb;
TRMS.H=TRMS.A*TRMS.lt+TRMS.B*TRMS.lm+(TRMS.mb*TRMS.lb*TRMS.lb/2)+TRMS.mcb*TRMS.lcb*TRMS.lcb;
%%
%% A predefined experiment

Exp.MRstep=1; Exp.MRini=0; Exp.MRfin=0.2; Exp.TRstep=30; Exp.TRini=0; Exp.TRfin=-2; Exp.tsim=60;


%%