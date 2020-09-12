function S = greedy_select_sensors(t,Pk,P_k_k_1,C,V,num_sensors)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here

U = 1:length(V);
S = [];
for i = 1:num_sensors
    best_index = 1;
    for j = 1:length(U)
       objective_S = objective_function(t,Pk,P_k_k_1,C,V,S,false);
       if objective_function(t,Pk,P_k_k_1,C,V,[S,U(best_index)],false)-objective_S <objective_function(t,Pk,P_k_k_1,C,V,[S,U(j)],false)-objective_S
           best_index = j;
       end
    end
    S  = [S,U(best_index)];
    S = sort(S);
    U(best_index) = [];
end

end

