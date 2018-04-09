%% Reference Load Profile for the COnsumer Demand
% This load is a reduced load that allows faster convergence

% Uref- kW 
% T   - hr 
function [Uref]=LoadRef2(t)

load ReducedLoad.mat

Uref=interp1(T,Ref_L_S,t,'linear','extrap');

end
