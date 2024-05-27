clc; clear; close all;
data = readtable('data.csv', 'Delimiter', ',');
data = table2array(data);

z_accel = data(:,1); %without the gravity factor
height = data(:,2);

% starting error and updating bme data
starting_error = mean(height(1:10,1));
height = height - starting_error;

Covariance_z_accel = cov(z_accel);
Covariance_height = cov(height);

disp("The covariance of accelrometer data is:");
disp(Covariance_z_accel);
disp("The covariance of height data is:")
disp(Covariance_height);
