%% Reference Load Profile for the COnsumer Demand
% This load is a step load for 50kW
% Uref- kW 
% T   - hr 

function [Uref]=LoadRef(T)
Uref=50*ones(1,numel(T));
end
