function [cost] = CostFn(U,x0k,S,W,t0k,nop,hopt,etaC,Pload_ref,time_ref)
%Computes the Cost corresponding to the set of inputs chosen given

%% Unpacking data
u1=U(1:2,:);
u2=U(3:4,:);
v =U(5  ,:);

%% Initialization
nH=1/5;         % Increase to make ode solve finer
xk=x0k;
tk=t0k;
cost=0;
for k=0:nop-1
    % Time Vector
    T=t0k+ hopt*(k :nH: (k+1));       % Time: [t0k:hsim:t0k+hopt]
        
    % Find Embedded System values
    u=v(k+1)*u1(:,k+1)+ (1-v(k+1))*u2(:,k+1);
    A=v(k+1)*S.A1     + (1-v(k+1))*S.A2;
    
    % Solving State Trajectory
    try
    [~,xk]=ode45(@(t,x) SysDyn(t,x,A,S.B,u,etaC),T,xk(:,end));
    catch
       breakpointhere =1;
    end
    xk=xk';
    
    % State Constraints
    % Upper Bounds
    cost = cost + W.Qvio*sum( (xk(1,:)>W.xub(1)).*(xk(1,:)-W.xub(1)).^2);  % For Ess

    % Lower Bounds
    cost = cost + W.Qvio*sum( (xk(1,:)<W.xlb(1)).*(xk(1,:)-W.xlb(1)).^2);  % For Ess
    
try
    % Find Reference Trajectory
    a(1)=find(T(1)==time_ref);          % Find index of start of time in given ref
    a(2)=find(T(end)==time_ref);        % Find index of end of time in given ref
    uload_ref=Pload_ref(a(1):a(2));     % Pick reference values from the given ref

    % Penalty
    Pload_err = (xk(2,:)+xk(3,:)- uload_ref);                               % Generation-Consumption=(Pess+Pd-Pwl-Pload)
    cost      = cost + W.Qtrack * sum(Pload_err.^2)/W.xub(3)^2;             % Load Tracking Penalty
    cost      = cost + W.Qdiesel* sum(xk(3,:).^2)/W.xub(3)^2;               % Diesel Use Penalty
    cost      = cost + W.QEss*sum( ((xk(1,:) - 0.9*W.xub(1))/W.xub(1)).^2); % Ess nominal deviation penalty
catch
    breakpointhere=1;    
end

end



end

