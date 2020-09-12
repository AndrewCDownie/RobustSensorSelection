function [H] = select_sensors(C ,sensors)

%generate the H matrix to use in Kalman filer based on selected sensors
H= [];
sensors = sort(sensors);
for i= 1:length(sensors)
   H = [H;C(sensors(i),:)];
end

end

