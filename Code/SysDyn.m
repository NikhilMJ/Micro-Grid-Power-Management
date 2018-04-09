function [dx]=SysDyn(t,x,A,B,u,etaC)


u   = u + etaC*[Psolar(t); 0;];
try
dx  = A*x + B*u;

% dx(3)= A(3,3)*x(3) + A(3,2)*max((x(2)+u(1)-u(3)),0);

catch
    breakpointhere=1
end

end