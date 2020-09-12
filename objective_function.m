function [value] = objective_function(t,Pk,P_k_k_1,C,V,sensors,error)
value = 0;
if length(sensors) == 0
    return
end

for i =1 :t-1
    value = value + trace(Pk(:,:,i));
end
Vs = select_measurements(V,sensors);
H = select_sensors(C,sensors);
value = value + trace(inv(inv(P_k_k_1(:,:,t))+ H'*inv(Vs)*H));
if error ==false
    value = -value;
end
end

