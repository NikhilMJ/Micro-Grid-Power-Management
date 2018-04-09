%% Reference Load Profile for the COnsumer Demand
% This load is as per H. Wu, X. Liu, M. Ding, Dynamic economic dispatch of a microgrid: Mathematical models and solution algorithm, Elec. Power and Energy Sys. 63, pp336-46, 2014


% Uref- kW 
% T   - hr 
function [Uref]=LoadRef1(T)

load LOAD_24h.mat
Uref=interp1(t,Load,T,'linear','extrap');

end
