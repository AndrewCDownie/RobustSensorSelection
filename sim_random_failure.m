function [obj_rand,obj_greedy,obj_ram,X,Xr,Xg,Xram,error_rand,error_greedy,error_ram] = sim_random_failure(time_size,n,A,C,V,M_start,V_start,M_end,V_end, start_loc,end_loc,k,beta)
 %set up recording Variables
 obj_rand = zeros(1,time_size);
 obj_greedy = zeros(1,time_size);
 obj_ram = zeros(1,time_size);

 
  
 error_rand = zeros(1,time_size);
 error_greedy = zeros(1,time_size);
 error_ram = zeros(1,time_size);
 
 %set up state variables
 X = zeros(n,time_size);
 Xr = zeros(n,time_size);
 Xg = zeros(n,time_size); 
 Xram = zeros(n,time_size); 



 %set intial conditions
 X(:,1) = [start_loc(1);
     (end_loc(1)- start_loc(1))/time_size;
     start_loc(2);
     (end_loc(2)- start_loc(2))/time_size;
     start_loc(3);
     (end_loc(3)- start_loc(3))/time_size;
     ];
 %all start with same intial condition
 Xr(:,1) = [M_start(1);
            (M_end(1)- M_start(1))/time_size;
            M_start(2);
            (M_end(2)- M_start(2))/time_size;
            M_start(3);
            (M_end(3)- M_start(3))/time_size;
           ];
 Xg(:,1) = Xr(:,1);
 Xram(:,1) = Xr(:,1);
 
 %set up covarience matrices
 Pr_k_k_1 = zeros(n,n,time_size);
 Pg_k_k_1 = zeros(n,n,time_size);
 Pram_k_k_1 = zeros(n,n,time_size);
 %sum of normal distributions are normaly distributed with means and
 %variences summed
 
 %All start with same inital Conditions
 Pr_k_k_1(:,:,1) = diag([V_start(1,1),
                        (V_start(1,1)+V_end(1,1))/time_size,
                         V_start(2,2),
                        (V_start(2,2)+V_end(2,2))/time_size,
                         V_start(3,3),
                        (V_start(3,3)+V_end(3,3))/time_size]);
 Pg_k_k_1(:,:,1) = Pr_k_k_1(:,:,1);
 Pram_k_k_1(:,:,1)= Pr_k_k_1(:,:,1);

 %initalize error covarience
 Pr = zeros(n,n,time_size);
 Pg = zeros(n,n,time_size);
 Pram = zeros(n,n,time_size);
 
 
 %Generate trajectory
 for t = 1:time_size
    %evolve the state
    X(:,t+1) = A*X(:,t);
 end
 
 %run Random sensor selection simulations
 for t = 1:time_size
     %select Sensors
    sensors = sort(datasample(1:100,k,'Replace',false));
    % remove sensors
    reduced_sensors = sort(datasample(sensors,k-beta,'Replace',false));
    % set up matrices based on selection
    H = select_sensors(C,reduced_sensors);
    Vs = select_measurements(V,reduced_sensors);
    
    Y = H*X(:,t) + mvnrnd(zeros(1,k-beta),Vs,1)';
    %K = Pk|k-1*H'(H*Pk|k-1*H' + Vs)^-1
    K = Pr_k_k_1(:,:,t)*H'*inv(H*Pr_k_k_1(:,:,t)*H' +Vs);
    
    %xhat_k+1 = A*xhat_k + K*(y-H*A*xhat_k)
    Xr(:,t+1) = A*Xr(:,t) + K*(Y-H*Xr(:,t));
    
    %Pk = (Pk|k-1^-1 + H'*Vs^-1*H)^-1
    Pr(:,:,t) = inv(inv(Pr_k_k_1(:,:,t)) + H'*inv(Vs)*H);
    
    %Pk+1|k = A*Pk*A'
    Pr_k_k_1(:,:,t+1) = A*Pr(:,:,t)*A';
    
    obj_rand(t) = objective_function(t,Pr,Pr_k_k_1,C,V,reduced_sensors,true);
    error_rand(t) = trace(Pr(:,:,t));
 end
 
  %run greedy Selection
 for t = 1:time_size
    %Select Sensors
    sensors = greedy_select_sensors(t,Pg,Pg_k_k_1,C,V,k);
    
    %remove sensors
    reduced_sensors = sort(datasample(sensors,k-beta,'Replace',false));
    
    %set up matrices from selection of sensors
    H = select_sensors(C,reduced_sensors);
    Vs = select_measurements(V,reduced_sensors);
    
    Y = H*X(:,t) + mvnrnd(zeros(1,k-beta),Vs,1)';
    %K = Pk|k-1*H'(H*Pk|k-1*H' + Vs)^-1
    K = Pg_k_k_1(:,:,t)*H'*inv(H*Pg_k_k_1(:,:,t)*H' +Vs);
    
    %xhat_k+1 = A*xhat_k + K*(y-H*A*xhat_k)
    Xg(:,t+1) = A*Xg(:,t) + K*(Y-H*Xg(:,t));
    
    %Pk = (Pk|k-1^-1 + H'*Vs^-1*H)^-1
    Pg(:,:,t) = inv(inv(Pg_k_k_1(:,:,t)) + H'*inv(Vs)*H);
    
    %Pk+1|k = A*Pk*A'
    Pg_k_k_1(:,:,t+1) = A*Pr(:,:,t)*A';
    obj_greedy(t) = objective_function(t,Pg,Pg_k_k_1,C,V,reduced_sensors,true);
    error_greedy(t) = trace(Pg(:,:,t));
 end
 
 %run RAM Selection
 for t = 1:time_size
    %Select Sensors
    sensors = RAM_select_sensors(t,Pram,Pram_k_k_1,C,V,k,beta);
    
    %Remove Sensors
    reduced_sensors = sort(datasample(sensors,k-beta,'Replace',false));
    
    %set up matrices from selection of sensors
    H = select_sensors(C,reduced_sensors);
    Vs = select_measurements(V,reduced_sensors);
    
    Y = H*X(:,t) + mvnrnd(zeros(1,k-beta),Vs,1)';
    
    %K = Pk|k-1*H'(H*Pk|k-1*H' + Vs)^-1
    K = Pram_k_k_1(:,:,t)*H'*inv(H*Pram_k_k_1(:,:,t)*H' +Vs);
    
    %xhat_k+1 = A*xhat_k + K*(y-H*A*xhat_k)
    Xram(:,t+1) = A*Xram(:,t) + K*(Y-H*Xram(:,t));
    
    %Pk = (Pk|k-1^-1 + H'*Vs^-1*H)^-1
    Pram(:,:,t) = inv(inv(Pram_k_k_1(:,:,t)) + H'*inv(Vs)*H);
    
    %Pk+1|k = A*Pk*A'
    Pram_k_k_1(:,:,t+1) = A*Pram(:,:,t)*A';
    
    obj_ram(t) = objective_function(t,Pram,Pram_k_k_1,C,V,reduced_sensors,true);
    error_ram(t) = trace(Pram(:,:,t));
 end

 





end
