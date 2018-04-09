%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Micro Grid Energy Optimization: MPC run on Real world Power Demand.
%% With inequality constraints. Uses projected mode and has inequality constraints built in
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Housekeeping
clc
clear
close all
filename=['Results_',strrep(strrep(strrep(datestr(now),':','_'),...
    '-','_'),' ','_')];     % Record time at which simulation started


%% User Inputs
clc

% MPC Parameters
t0          = 0;           % hr  Start Time 
tf          = 24;           % hr  End Time 
nPart       = 8;            % part Horizon Window
hopt        = 0.5;          % hr  Optimization time step
nH          = 1/5;           %     Indicates fineness of cost-fn evaluation.
N           = ceil(tf/hopt);%      Total number of steps to be taken;

% System Parameters
deltaEss   =  0.04;        % %    Discharge percentage for Ess per hr
tD         =  0.3;         % hr   Time constant for Diesel Gen
tEss       =  0.1;         % hr   Time constant for Pess
 
etaC       = 0.9;          %  -   Charging Eff
etaD       = 0.9;          %  -   Discharging Eff

% State Constraints
PDmax      = 150;          % kW   Max power of the generator
PDmin      = 0;            % kW   Min power of the generator (Spinning reserve)

Ucharge    =  200;         % kW   Max charging power to battery Ess
Udischarge =  200;         % kW   Max discharging power to battery Ess
 
Ess_ub      =1;           % kWh    Max capacity of battery
Ess_lb      =0.9*Ess_ub;  % kWh    Min allowed ESS
Ess_nom     =800;        % kWh    Max capacity of Ess

Pess_ub     = Ucharge;    % kW     Max charging power (charging)
Pess_lb     =-Udischarge; % kW     Min charging power (dis-charging)

Pd_ub       =PDmax;       % kW     Max Power output of the Diesel Gen
Pd_lb       =PDmin;       % kW     Min Power (Spinning reserve)

% Compile the State Bounds
W.xub      =[Ess_ub; Pess_ub;  Pd_ub;];  % Upper State bound
W.xlb      =[Ess_lb; Pess_lb;  Pd_lb;];  % Lower State bound

% Initial State
x0          =[0.99*W.xub(1); Pess_ub; 1;]; 

% System Information=> Stored in 'S' structure
% xdot= (vA1 + (1-v)A2) x + B.(vU1 + (1-v)U2)
S.A1=[-deltaEss    -1/etaD/Ess_nom     0      ;
         0         -1/tEss     0      ;
         0           0           -1/tD    ;];
     
S.A2=[-deltaEss     -etaC/Ess_nom     0      ;
         0         -1/tEss      0      ;
         0            0      -1/tD    ;];
          
S.B= [   0           0      ;
       1/tEss        0      ;
         0          1/tD    ;];
cntrl1=ctrb(S.A1,S.B);
cntrl2=ctrb(S.A2,S.B);
cond_cntrl1=cond(cntrl1)
cond_cntrl2=cond(cntrl2)



%% Input Bounds

% Mode 0- Discharge
U0_ub       = [Udischarge ;Pd_ub];
% U0_lb       = [0-5          ;Pd_lb];
U0_lb       = [-Ucharge   ;Pd_lb];

% Mode 1- Charge
U1_ub       = [Udischarge ;Pd_ub];
% U1_ub       = [0+5        ;Pd_ub];
U1_lb       = [-Ucharge   ;Pd_lb];


% Compile the Input Bounds
Ulb         = [U0_lb; U1_lb; 0];
Uub         = [U0_ub; U1_ub; 1];


LB          = repmat(Ulb,1,nPart);
UB          = repmat(Uub,1,nPart);


% Initial Input
Uguess      = UB;         %        Initial Guesses for one iteration

% Forming input constraints for fmincon
Aeq=[1 1 0 0 0;
     0 0 1 1 0;];
 
Z=zeros(size(Aeq));
A=[];
for ii=0:nPart-1
    A=[A; [repmat(Z,1,ii) Aeq repmat(Z,1,nPart-1-ii)] ];
end 
Aeq=A;


%% Cost Weights
W.Q         =diag(1/W.xub);
W.R         =diag(1/[Uub]);
W.Qvio      =100;           % State violation cost
W.Qtrack    =40;            % Load Tracking Cost
W.Qdiesel   =5;             % Diesel Fuel usage cost
W.QEss      =2;             % Deviation from nominal cost


%% Initializations for MPC
x0k    =x0; 
xk     =x0;
xk_save=[];
t_save =[];
u_save =[];
v_save =[];
T_save=0;

% FMINCON Initialization
Tol=1e-2;
options=optimset('Algorithm','active-set','Display','off'...
    ,'MaxIter',1000,'MaxFunEvals',1e3...
    ,'TolFun',Tol,'TolX',Tol...
    );

tic

embed=2;
for ii=1:N    
    % Find Initial time of Window
    t0k       = t0+(ii-1)*hopt;
    T         = t0k+ hopt*(0 :nH: (nPart));       % Time Vector
    Pload_ref = LoadRef1(T);
    Beq       = mean(Pload_ref)*[1;1];
    Beq       = mean(Pload_ref)*[1;1]+10;         % Add 10kW
    Beq       = repmat(Beq,nPart,1);
    
    % Fmincon Optimization
%     [Uopt,J(ii),flag(ii),output(ii)] =...
%         fmincon( @(U) CostFn(U,x0k,S,W,t0k,nPart,hopt,etaC,Pload_ref,T),...
%         Uguess,[],[],Aeq,Beq,LB,UB,[],options);
    
    [Uopt,J(ii),flag(ii),output(ii)] =...
        fmincon( @(U) CostFn(U,x0k,S,W,t0k,nPart,hopt,etaC,Pload_ref,T),...
        Uguess,Aeq,Beq,[],[],LB,UB,[],options);
    
    % Unpacking Fmincon output
    u1=Uopt(1:2,:);     u1part=u1(:,1);
    u2=Uopt(3:4,:);     u2part=u2(:,1);
    v =Uopt(5  ,:);      vpart= v(:,1);
    

    % Define hybrid switching methodloy i.e. either a Sliding mode or projected mode
    % Embed=0:  Sliding mode embeds a linear combination of the hybrid states and the inputs for each state
    % Embed=1:  Slides the inputs for each state and since input +/- drives the state, projection is done later
    % Embed=2:  Both input and state models are projected to either of the modes
    
     % Finding embedded input/system matrices
     if(embed==0)
             upart = vpart*u1part + (1-vpart)*u2part;
             Apart = vpart*S.A1   + (1-vpart)*S.A2;

     % embedding only input and projecting system model
     elseif(embed==1)
         upart = vpart*u1part + (1-vpart)*u2part;
         if(upart<0)
             upart=u2part;
             Apart=S.A2;
             vpart=1;
         else
             upart=u1part;
             Apart=S.A1;
             vpart=0;
         end 

     % projecting both the input and system model    
     elseif(embed==2)
         if(vpart<0.5)
             upart=u1part;
             Apart=S.A1;
             vpart=0;
         else
             upart=u2part;
             Apart=S.A2;
             vpart=1;
         end
         
     end
    % Simulate Real System
    T       =t0k+ hopt*(0:0.1:1);           % Time: [t0k:hsim:t0k+hopt]
    [t,xk]  =ode23t(@(t,x) SysDyn(t,x,Apart,S.B,upart,etaC),T,xk(:,end));
    xk      =xk';
    
    % Reinitilizations
    x0k     =xk(:,end);                     % Updating Initial State
    Uguess  =[Uopt(:,2:end) Uopt(:,end)];   % Reinitializing the Initial Guess
    
    % Saving Stuff
    t_save  =[t_save  t'];
    xk_save =[xk_save xk];
    u_save  =[u_save  upart ];
    v_save  =[v_save  vpart ];
    T_save  =[T_save  T(end)     ];
    
    % Display Loop Information
    clc
    disp(['Percent Done: ',num2str((ii/N)*100)]);
    disp(['Time elapsed: ',num2str(toc),'sec']);
    disp(['   Exit Flag: ',num2str(flag(ii))]);
    disp(['  Cost Value: ',num2str(J(ii))]);
end

%% Saving Workspace
mkdir(filename);                        % Create directory for saving results
address=[pwd '\' filename '\' ];        % Find overall address to directory
save([address filename '.mat']);        % Save the results

%% Plotting
close all

% States
h1=figure;
h(1) = subplot(4,1,1);
plot(t_save,xk_save(1,:),'displayname','Ess [Normalized]');
ylabel('SOC Normalized [-]')
legend('show');

h(2) = subplot(4,1,2);
plot(t_save,xk_save(2,:),'displayname','Pess [kW]');
ylabel('Power [kW]')
legend('show');

h(3) = subplot(4,1,3);
plot(t_save,xk_save(3,:),'displayname','Pd [kW]');
ylabel('Power [kW]')
legend('show');

h(4)=subplot(4,1,4);
plot(t_save,LoadRef2(t_save),'displayname','Reference Load Profile [kW]');
ylabel('Power [kW]')
xlabel('Time [Hr]');
legend('show');
linkaxes(h,'x')

% Inputs
h2=figure;
T= t0:hopt:tf-hopt;
h(5) = subplot(3,1,1);
stairs(T_save(1:end-1),u_save(1,:),'displayname','Uess [kW]');
ylabel('SS Power Command [kW]')
legend('show');

h(6) = subplot(3,1,2);
stairs(T_save(1:end-1),u_save(2,:),'displayname','Udiesel [kW]');
ylabel('DG Power Command [kW]')
legend('show');

h(8) = subplot(3,1,3);
stairs(T_save(1:end-1),v_save,'displayname','Mode');
stairs(T_save(1:end-1),u_save(1,:)<=0,'displayname','Mode');
legend('show');
axis([0 24 -0.1 1.1])
xlabel('Time [hrs]');
ylabel('Mode');
set(gca,'YTick',[0 1])
set(gca,'YTickLabel',['Discharge';'Charge   '])
linkaxes(h,'x')

% PLotting Cost Function
h3=figure;
subplot(2,1,1);
plot(t_save, xk_save(2,:)+xk_save(3,:)-LoadRef2(t_save))
subplot(2,1,2);
plot(cumsum(J),'displayname','Iteration Cost');
legend('show');

%% Saving Figures as PNG
% Saving Figures
print(h1,'-dpng',[address 'State_Trajectory.png']);
print(h2,'-dpng',[address 'Inputs_Applied.png']);
print(h3,'-dpng',[address 'Cost_Fn.png']);
