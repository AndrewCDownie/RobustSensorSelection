function S = RAM_select_sensors(t,Pk,P_k_k_1,C,V,k,beta)

U = 1:length(V);%indice vector
S = [];%Selection Set

%Find the k-beta best elements

for i = 1:k-beta
    best_index = 1;
    best_value = objective_function(t,Pk,P_k_k_1,C,V,[U(best_index)],false);
    for j = 1:length(U)
        val = objective_function(t,Pk,P_k_k_1,C,V,[U(j)],false);
        if  val > best_value
            best_index = j;
            best_value = val;
        end
    end
    S = sort([S,U(best_index)]);
    U(best_index) = [];      
end

%fill the rest of the elements by greedly selecting elements
for i = 1:beta
    best_index = 1;
    for j = 1:length(U)
       objective_S = objective_function(t,Pk,P_k_k_1,C,V,S,false);
       if objective_function(t,Pk,P_k_k_1,C,V,[S,U(best_index)],false)-objective_S <objective_function(t,Pk,P_k_k_1,C,V,[S,U(j)],false)-objective_S
           best_index = j;
       end
    end
    S  = sort([S,U(best_index)]);
    U(best_index) = [];
    
end



end

