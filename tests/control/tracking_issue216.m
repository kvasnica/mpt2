function tracking_issue216

% NOTE! read discussion to issue216:
%  http://autsun04.ee.ethz.ch:8080/mpt/issue216
%
% one of the important things tested here is runtime - there was a performance
% regression in the official 2.6 release, fixed mainly by
%   http://control.ee.ethz.ch/~mpt/hg/mpt-26x?cs=ba9444eb2e42
%
% the runtime for this test is currently around 3 secs using NAG, or 22 secs
% using quadprog

% % clear all
% % close all
close all
clear

load boilerdata_R13

% Defining model dimentions
nu = size(B,2);
nd = size(Bd,2);
nx = size(A,1);
ny = size(C,1);
nz = size(C,1);
%%%%%%%%%%%%%%%%%

% Collecting input and disturbance vector 
model=ss(A,[B Bd],C,[D Dd]);
%%%%%%%%%%%%%%%%%

% Dicritization of model
Ts=1; Bsys.Ts=Ts;
modeld = c2d(model,Ts);
%%%%%%%%%%%%%%%%%
% Plant model
usplant.A = modeld.a;
usplant.B = modeld.b(:,1:nu);
usplant.Bd = modeld.b(:,1+nu:nu+nd);
usplant.C = modeld.c;
usplant.D = modeld.d(:,1:nu);
usplant.Dd = modeld.d(:,1+nu:nu+nd);
%%%%%%%%%%%%%%%%%

% Defining scaling parameters
Du=diag([Qp_max mfw_max]); % input scaling matrix
Dd=diag([k_max]); % disturbace scaling matrix
De=diag([Ps_max Lw_max]); % control error scaling matrix
T=diag([Ps_max Vw_max Vb_max]); % state scaling matrix

scale.Du=Du;
scale.Dd=Dd;
scale.De=De;
scale.Dr=De;

scale.T=T;
% System norming/scaling matrices
modeld=De^(-1)*ss2ss(modeld,T^(-1))*diagmx(Du,Dd);
%%%%%%%%%%%%%%%%%

% Plant model
plant.A = modeld.a;
plant.B = modeld.b(:,1:nu);
plant.Bd = modeld.b(:,1+nu:nu+nd);
plant.C = modeld.c;
plant.D = modeld.d(:,1:nu);
plant.Dd = modeld.d(:,1+nu:nu+nd);

plant.nu = size(plant.B,2);
plant.nd = size(plant.Bd,2);
plant.nx = size(plant.A,1);
plant.ny = size(plant.C,1);
%%%%%%%%%%%%%%%%%

% estimator definition

% augmenting state equations by unmeasured output disturbance to
% achieve offset free tracking by providing unbiased estimates of output
% x(k+1) = Aaug*x(k)+Baug*u(k)+Gaug*nx(k)
% ym(k) = Caug*x(k)+Daug*u(k)+Haug*nx(k)+nym(k)

% integrated white noise model for unmeasured output disturbance
% $$$ [Aumd,Bumd,Cumd,Dumd]=c2dm(zeros(plant.ny-1),eye(plant.ny-1), ...
% $$$     eye(plant.ny-1),zeros(plant.ny-1),Ts);
% $$$ Unmeasured output disturbance
% $$$ Aaug = diagmx(plant.A,Aumd);
% $$$ Baug = [[plant.B plant.Bd];zeros(plant.ny-1,plant.nu+plant.nd)];
% $$$ Caug = [plant.C,[Cumd;0]];
% $$$ Daug = [plant.D plant.Dd];
% $$$ Gaug = [zeros(plant.nx,plant.ny-1) plant.B plant.Bd; ...
% $$$         Bumd zeros(plant.ny-1,plant.nu) zeros(plant.ny-1,plant.nd)];
% $$$ Haug = zeros(plant.ny,plant.ny-1+plant.nu+plant.nd);

% integrated white noise model for unmeasured input distrubance
[Aumd,Bumd,Cumd,Dumd]=c2dm(zeros(plant.ny),eye(plant.ny), ...
    eye(plant.ny),zeros(plant.ny),Ts);
Aaug = [plant.A plant.B*Cumd;zeros(plant.ny,plant.nx) Aumd];
Baug = [[plant.B plant.Bd];zeros(plant.ny,plant.nu+plant.nd)];
Caug = [plant.C,zeros(plant.ny)];
Daug = [plant.D plant.Dd];
Gaug = [zeros(plant.nx,plant.ny) plant.B plant.Bd; ...
  Bumd zeros(plant.ny,plant.nu) zeros(plant.ny,plant.nd)];
Haug = zeros(plant.ny,plant.ny+plant.nu+plant.nd);

% observability
Mobs=obsv(Aaug,Caug);
[row,col]=size(Mobs);
if (rank(Mobs)<min(row,col))
  error(['System not observable'])
end

% Noise covariance for nx and nym
Q=eye(plant.ny+plant.nu+plant.nd); R=eye(plant.ny);
sysaug=ss(Aaug,[Baug Gaug],Caug,[Daug Haug],Ts);
[dummy1,dummy2,dummy3,M]=kalman(sysaug,Q,R);

Best.A = [Aaug-Aaug*M*Caug];
Best.B = [Baug Aaug*M];
Best.C = [eye(size(Aaug,1))-M*Caug];
Best.D = [zeros(size(Aaug,1),size(Baug,2)) M];

% measered disturbance model
[Amd,Bmd,Cmd,Dmd]=c2dm(zeros(plant.nd),zeros(plant.nd), ...
    ones(plant.nd),zeros(plant.nd),Ts);

% sysStruct definition
% This is the plant model augmented by the unmeasured disturbances model
% for achieving offset free tracking and the measured disturbace
% model (feed forward)
Bsys.A = [Aaug Baug(:,plant.nu+1:end)*Cmd;
           zeros(plant.nd,plant.nx+plant.ny) Amd]; 
Bsys.B = [Baug(:,1:plant.nu);zeros(plant.nd,plant.nu)]; 
Bsys.C = [Caug zeros(plant.ny,plant.nd)]; 
Bsys.D = Daug(:,1:plant.nu);
Bsys.InputName = {'Qp', 'mfw'};
Bsys.OutputName = {'ps', 'Lw'};
Bsys.StateName = {'ps','Vw','Vb','uumd1','uumd2','ymd1'};

% system constraints with scaling
Bsys.umin = Du^(-1)*[Qpmin;mfwmin];
Bsys.umax = Du^(-1)*[Qpmax;mfwmax];

Bsys.dumin = Du^(-1)*[dQpmin;dmfwmin];
Bsys.dumax = Du^(-1)*[dQpmax;dmfwmax];

Bsys.ymin = De^(-1)*[Psmin;Lwmin];
Bsys.ymax = De^(-1)*[Psmax;Lwmax];

% Defining set for which the states are contained in
Bsys.xmax=100*ones(size(Bsys.A,1),1);
Bsys.xmin=-Bsys.xmax;
% $$$ Phk=polytope([eye(length(xmax));-eye(length(xmax))],[xmax;xmax]);
% $$$ Bsys.Pbnd=Phk;
Bsys.Pbnd = unitbox(size(Bsys.A,1), 10);

% % Bsys=mpt_verifySysStruct(Bsys);

% construction of probStruct
disp('Defining probStruct');
Bprob.subopt_lev = 0;
Bprob.norm = 2; % problem norm
Bprob.N = 20; % prediction horizon
Bprob.Nc =2; % control horizon/blocking
% Bprob.inputblocking=[1 Bprob.N-1];
Bprob.Q = eye(size(Bsys.A,1))^2; % weigths on states
% If no weigths specifyied the identity matrix is used
Bprob.Qy =1e0*diag([1;1])^2; % weigths on outputs
Bprob.R = 1e1*diag([1;1])^2; % weigths on inputs
Bprob.P_N = Bprob.Qy; % terminal weighting different from the 
                      % solution of the Ricatti equation 
Bprob.y0bounds=0; % do not put weight on y(0) in performance index

Bprob.tracking = 1; % tracking with deltau formulation

% probStruct definition
Bprob=mpt_verifyProbStruct(Bprob);

% construction of ctrlStruct
disp('Defining ctrlStruct');

Bctrl = mpt_control(Bsys,Bprob, 'on-line');

% constraints in MPT toolbox:
% [Dumax/umax(1)]
% [Dumin/umin(1)]
% [ymax(1)      ]
% [ymin(1)      ]
% [      :      ]
% [Dumax/umax(N)]
% [Dumin/umin(N)]
% [ymax(N)      ]
% [ymin(N)      ]
% [-------------]
% [xmax         ]
% [xmin         ]
%
% Remember here that in tracking problems y=[y u r]' and x=[x u r]'.
% Also not that even though you use input blocking this does not
% reduce the number of constraints.


% simulation
disp('Now simulating nominal closed-loop behavior');
%%tic
Nsteps = 60; % 60
TimeSteps = round(Nsteps*10/Ts);
T = [0:Ts:Ts*(TimeSteps-1)];
R = zeros(2,1)*ones(1,TimeSteps);
% disturbance corrosponding to a step in steam flow disturbance of -1000kg/h
D = [zeros(50/Ts,1);ones(TimeSteps-50/Ts,1)]'*(-1000/3600)/sqrt(7e5);

Y=zeros(size(plant.C,1),TimeSteps);
X=zeros(size(plant.A,1),TimeSteps);
U = zeros(size(plant.B,2),TimeSteps); 
Xbar=zeros(size(Best.A,1),TimeSteps);
Xhat=zeros(size(Best.A,1),TimeSteps);

for k=1:TimeSteps
  % Plant equations: output equation
  Y(:,k) = usplant.C*X(:,k);
  
  % Estimator equations: state correction
  Xhat(:,k) = Best.C*Xbar(:,k)+ ...
      Best.D*[Du^(-1)*U(:,k);Dd(1,1)^(-1)*D(:,k);De^(-1)*Y(:,k)];
  
  % Compute MPC law
  if k==1
    Uold = zeros(plant.nu,1);
  else
    Uold = U(:,k-1);
  end

  x0 = [Xhat(:,k);Dd(1,1)^(-1)*D(:,k);Du^(-1)*Uold;De^(-1)*R(:,k)];
  uu = mpt_getInput(Bctrl,x0);
  U(:,k) = Du*uu;
  if Bprob.tracking==1,
      U(:,k) = U(:,k)+Uold;
  end
  
  % Plant equations: state update
  if k<TimeSteps 
    X(:,k+1) = usplant.A*X(:,k)+usplant.B*U(:,k)+usplant.Bd*D(:,k); 
    Xbar(:,k+1) = Best.A*Xbar(:,k)+ ...
	Best.B*[Du^(-1)*U(:,k);Dd(1,1)^(-1)*D(:,k);De^(-1)*Y(:,k)];     
  end
end
%%toc

mbg_asserttolequal(Xhat(:, end), [0;2.4598;-2.4598;0;0], 1e-4);
mbg_asserttolequal(Y(:, end), [0; 0]);
% 
% subplot(221)
% plot(T, 1e-5*Y(1,:), T, 1e-5*R(1,:), ...
%      [T(1);T(end)],1e-5*[Psmax Psmin;Psmax Psmin],'r'),grid
% title('Pressure [bar]');
% subplot(223)
% plot(T, 1e3*Y(2,:), T, 1e3*R(2,:), ... 
%      [T(1);T(end)],1e3*[Lwmax Lwmin;Lwmax Lwmin],'r'),grid
% title('Water level [mm]')
% subplot(222)
% plot(T,1e-3*U(1,:), ...
%      [T(1);T(end)],1e-3*[Qpmax Qpmin;Qpmax Qpmin],'r'),grid
% title('Energy input [KJ/s]')
% subplot(224)
% plot(T,3600*U(2,:),T,3600*(D.*sqrt(Y(1,:)+7e5)), ...
%      [T(1);T(end)],3600*[mfwmax mfwmin;mfwmax mfwmin],'r'),grid
% title('Feed water input, Steam flow disturbance')
