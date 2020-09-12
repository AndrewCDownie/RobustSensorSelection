function U = greedy_remove_sensors(t,Pk,P_k_k_1,C,V,sensors,beta)


%Remove the sensors with largest marginal returns
U = sensors;
S = [];
for i = 1:beta
    best_index = 1;
    for j = 1:length(U)
       objective_S = objective_function(t,Pk,P_k_k_1,C,V,S,false);
       if objective_function(t,Pk,P_k_k_1,C,V,[S,U(best_index)],false)-objective_S <objective_function(t,Pk,P_k_k_1,C,V,[S,U(j)],false)-objective_S
           best_index = j;
       end
    end
    S = [S,U(best_index)];
    S = sort(S);
    U(best_index) = [];
end

end

