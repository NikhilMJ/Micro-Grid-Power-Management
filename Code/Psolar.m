function [Psolar_ref] = Psolar(t)
% Returns the value of the solar power being generated at a given time 't'
% Psolar_ref: kW 
persistent PV_data;
if(isempty(PV_data))
    load PV_Solar_data.mat;
    PV_data=PV_Solar_data;
end

Psolar_ref=0;
% Uncomment for solar data
% Psolar_ref=interp1(PV_data(:,1),PV_data(:,2),t,'linear','extrap');


end