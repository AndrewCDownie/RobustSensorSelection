%Andrew Downie
%ECE 686 Project Code
%Robust and Adaptive Sequential Submodular Optimization
%

clear;
clf;
close all;
rng(102);


%% define state space system
n = 6; %dimension of state

A = [0,1,0,0,0,0;
     0,0,0,0,0,0;
     0,0,0,1,0,0;
     0,0,0,0,0,0;
     0,0,0,0,0,1;
     0,0,0,0,0,0];

%distritize the the Continuous System
T = 1;
At = expm(A*T); 

time_size = 5;

X = zeros(n,time_size);
Xr = zeros(n,time_size);
Xg = zeros(n,time_size);
Xram = zeros(n,time_size);

% %set start and finish locations
M_start = [0,0,0];
V_start = 1*eye(3);
%start_loc = mvnrnd(M_start,V_start);

M_end = [10,10,10];
V_end = 1*eye(3);
%end_loc = mvnrnd(M_end,V_end);
% 
% %calculate the direction of the UAV
% vels = [(end_loc(1)-start_loc(1))/5,
%         (end_loc(2)-start_loc(2))/5,
%         (end_loc(3)-start_loc(3))/5];
%     
% %inital state
% X(:,1) = [start_loc(1);
%           vels(1);
%           start_loc(2);
%           vels(2);
%           start_loc(3);
%           vels(3);];
% 

%% define sensors
%number of sensors
m = 100;

%number of sensors to select at each time
k = 10;

%Generate C matrix
C =randn(m,n);

%get covarience matrix
%V = diag(10*randn(1,m).^2);




%% Simulate Exectution


%Number of trials that will be averaged over
%num_trials = 100;
num_trials = 25; %reduced the number of trials to run faster 

%betas to test with
betas = [3,5,7,9];

%create listed to record all the data from the tests
trials_rand_randomR = zeros(num_trials,time_size,length(betas));
average_rand_randomR = zeros(length(betas),time_size);
error_rand_randomR = zeros(num_trials,time_size,length(betas));

trials_rand_greedyR = zeros(num_trials,time_size,length(betas));
average_rand_greedyR = zeros(length(betas),time_size);
error_rand_greedyR = zeros(num_trials,time_size,length(betas));

trials_rand_worstR = zeros(num_trials,time_size,length(betas));
average_rand_worstR = zeros(length(betas),time_size);
error_rand_worstR = zeros(num_trials,time_size,length(betas));

trials_greedy_randomR = zeros(num_trials,time_size,length(betas));
average_greedy_randomR = zeros(length(betas),time_size);
error_greedy_randomR = zeros(num_trials,time_size,length(betas));

trials_greedy_greedyR = zeros(num_trials,time_size,length(betas));
average_greedy_greedyR = zeros(length(betas),time_size);
error_greedy_greedyR = zeros(num_trials,time_size,length(betas));

trials_greedy_worstR = zeros(num_trials,time_size,length(betas));
average_greedy_worstR = zeros(length(betas),time_size);
error_greedy_worstR = zeros(num_trials,time_size,length(betas));

trials_ram_randomR = zeros(num_trials,time_size,length(betas));
average_ram_randomR = zeros(length(betas),time_size);
error_ram_randomR = zeros(num_trials,time_size,length(betas));

trials_ram_greedyR = zeros(num_trials,time_size,length(betas));
average_greedy_greedyR = zeros(length(betas),time_size);
error_ram_greedyR = zeros(num_trials,time_size,length(betas));

trials_ram_worstR = zeros(num_trials,time_size,length(betas));
average_ram_worstR = zeros(length(betas),time_size);
error_ram_worstR = zeros(num_trials,time_size,length(betas));

%iterate over trials
tic


%loop over betas and generate simulation data 
for j = 1:length(betas)
    fprintf("Running Simulations for Beta = %d\n",betas(j))
    for i = 1:num_trials
        
        %get covarience matrix for trial
        V = diag(10*randn(1,m).^2);
        start_loc = mvnrnd(M_start,V_start);
        end_loc = mvnrnd(M_end,V_end);
        %run the simulations
        
        %worst case failure
        [obj_rand_worst,obj_greedy_worst,obj_ram_worst,Xw,Xrw,Xgw,Xramw,error_rand_worst,error_greedy_worst,error_ram_worst] = sim_worst_case_failure(time_size,n,At,C,V,M_start,V_start,M_end,V_end, start_loc,end_loc,k,betas(j));
        
        %random failure
        [obj_rand_random,obj_greedy_random,obj_ram_random,X,Xr,Xg,Xram,error_rand_random,error_greedy_random,error_ram_random] = sim_random_failure(time_size,n,At,C,V,M_start,V_start,M_end,V_end, start_loc,end_loc,k,betas(j));
        
        %greedy failure
        [obj_rand_greedy,obj_greedy_greedy,obj_ram_greedy,X,Xr,Xg,Xram,error_rand_greedy,error_greedy_greedy,error_ram_greedy] = sim_greedy_failure(time_size,n,At,C,V,M_start,V_start,M_end,V_end, start_loc,end_loc,k,betas(j));
        
        
        
        %record data for later averaging
        trials_rand_worstR(i,:,j) = obj_rand_worst;
        trials_greedy_worstR(i,:,j) = obj_greedy_worst;
        trials_ram_worstR(i,:,j) = obj_ram_worst;
        
        trials_rand_randomR(i,:,j) = obj_rand_random;
        trials_greedy_randomR(i,:,j) = obj_greedy_random;
        trials_ram_randomR(i,:,j) = obj_ram_random;
        
        trials_rand_greedyR(i,:,j) = obj_rand_greedy;
        trials_greedy_greedyR(i,:,j) = obj_greedy_greedy;
        trials_ram_greedyR(i,:,j) = obj_ram_greedy;
        
        error_rand_greedyR(i,:,j) = error_rand_greedy;
        error_greedy_greedyR(i,:,j) = error_greedy_greedy;
        error_ram_greedyR(i,:,j) = error_ram_greedy;
        
        error_rand_worstR(i,:,j) = error_rand_worst;
        error_greedy_worstR(i,:,j) = error_greedy_worst;
        error_ram_worstR(i,:,j) = error_ram_worst;
        
        error_rand_randomR(i,:,j) = error_rand_random;
        error_greedy_randomR(i,:,j) = error_greedy_random;
        error_ram_randomR(i,:,j) = error_ram_random;
        
        
        %save data for plot
        if betas(j) ==5
            if i == 1
                X = Xw;
                Xr = Xrw;
                Xg = Xgw;
                Xram = Xramw;
            end
        end
            
        
        
    end
    
end
toc

%Average the cumulative or error at each time
for i = 1:length(betas)
    for j = 1:time_size
        average_rand_worstR(i,j) = mean(trials_rand_worstR(:,j,i));
        average_greedy_worstR(i,j) = mean(trials_greedy_worstR(:,j,i));
        average_ram_worstR(i,j) = mean(trials_ram_worstR(:,j,i));
        
        average_rand_randomR(i,j) = mean(trials_rand_randomR(:,j,i));
        average_greedy_randomR(i,j) = mean(trials_greedy_randomR(:,j,i));
        average_ram_randomR(i,j) = mean(trials_ram_randomR(:,j,i));
        
        average_rand_greedyR(i,j) = mean(trials_rand_greedyR(:,j,i));
        average_greedy_greedyR(i,j) = mean(trials_greedy_greedyR(:,j,i));
        average_ram_greedyR(i,j) = mean(trials_ram_greedyR(:,j,i));

        
         %UNCOMMENT THIS SECTION IF YOU WANT TO COMPARE THE ERROR AT EACH
         %TIMESTAGE INSTEAD OF CUMMULATIVE ERROR
%         average_rand_greedyR(i,j) = mean(error_rand_greedyR(:,j,i));
%         average_greedy_greedyR(i,j) = mean(error_greedy_greedyR(:,j,i));
%         average_ram_greedyR(i,j) = mean(error_ram_greedyR(:,j,i));
%         
%         average_rand_randomR(i,j) = mean(error_rand_randomR(:,j,i));
%         average_greedy_randomR(i,j) = mean(error_greedy_randomR(:,j,i));
%         average_ram_randomR(i,j) = mean(error_ram_randomR(:,j,i));
%         
%         average_rand_worstR(i,j) = mean(error_rand_worstR(:,j,i));
%         average_greedy_worstR(i,j) = mean(error_greedy_worstR(:,j,i));
%         average_ram_worstR(i,j) = mean(error_ram_worstR(:,j,i));
    end
    
end


%Plotting 
for i = 1:length(betas)
    
    title_text = sprintf("Worst Case Removal Beta = %d",betas(i));
    file_name = sprintf("plots/Worst Case Removal Beta = %d.png",betas(i));
    fig = figure();
    title(title_text)
    hold on
    %uncomment to compare to random selection
    %plot(1:time_size,average_rand_worstR(i,:),'r')
    plot(1:time_size,average_greedy_worstR(i,:),'g')
    plot(1:time_size,average_ram_worstR(i,:),'m')
    %legend(["random","greedy","RAM"])
    legend(["greedy","RAM"])
    xlabel("T");
    ylabel("cumulative error");
    
    %Uncomment line below to save plot figures into plots folder
    %saveas(fig,file_name);
    
    
    title_text = sprintf("Random Removal Beta = %d",betas(i));
    file_name = sprintf("plots/Random Removal Beta = %d.png",betas(i));
    fig = figure();
    title(title_text)
    hold on
    
    %uncomment to compare to random selection
    %plot(1:time_size,average_rand_randomR(i,:),'r')
    
    plot(1:time_size,average_greedy_randomR(i,:),'g')
    plot(1:time_size,average_ram_randomR(i,:),'m')
    %legend(["random","greedy","RAM"])
    legend(["greedy","RAM"])
    xlabel("T");
    ylabel("cumulative error");
    
    %Uncomment line below to save plot figures into plots folder
    %saveas(fig,file_name);
    
    title_text = sprintf("Greedy Removal Beta = %d",betas(i));
    file_name = sprintf("plots/Greedy Removal Beta = %d.png",betas(i));
    fig = figure();
    title(title_text)
    hold on
    %uncomment to compare to random selection
    %plot(1:time_size,average_rand_greedyR(i,:),'r')
   
    plot(1:time_size,average_greedy_greedyR(i,:),'g')
    plot(1:time_size,average_ram_greedyR(i,:),'m')
    
    legend(["greedy","RAM"])
    xlabel("T");
    ylabel("cumulative error");
    
    %Uncomment line below to save plot figures into plots folder
    %saveas(fig,file_name);
end

figure

hold on
title("Example Tracking of UAV with beta = 5")
xlabel("x(m)")
ylabel("y(m)")
for i= 1:time_size
    plot(X(1,i),X(3,i),"*b");
    plot(Xg(1,i),Xg(3,i),"*g");
    plot(Xr(1,i),Xr(3,i),"*r");
    plot(Xram(1,i),Xram(3,i),"*m");
    legend("X","Greedy","Random","RAM")
    pause(2)
end


