function [Vs] = select_measurements(V, sensors)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
v = diag(V);
vs = [];
for i = 1:length(sensors)
    vs = [vs,v(sensors(i))];
end
Vs = diag(vs);
end

