function [worst_reduced_sensors] = remove_worst_case(t,Pk,P_k_k_1,C,V,sensors,beta)
%get all the combinations of removals
combos = nchoosek(1:length(sensors),beta);

worst_reduced_sensors = sensors;

worst_reduced_sensors(combos(1,:)) =[];

worst_reduced_val = objective_function(t,Pk,P_k_k_1,C,V,worst_reduced_sensors,false);

for i = 1:length(combos)
    reduced_sensors = sensors;
    reduced_sensors(combos(i,:)) =[];
    reduced_val = objective_function(t,Pk,P_k_k_1,C,V,reduced_sensors,false);
    if reduced_val <worst_reduced_val
        worst_reduced_val = reduced_val;
        worst_reduced_sensors = reduced_sensors;
    end
end

end

