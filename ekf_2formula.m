close all
clear all
drawArrow = @(x,y) quiver( x(1),y(1),x(2),y(2),0 )    

filename = 'gt_000002.txt';
data_format = '%d\t%d\t%f\t%f\n';
data_size = 4;

dt = 0.45; %seconds
err_x = 0.0001;
err_y = 0.0001;
n_state = 5;

P = eye(n_state);
R = [err_x 0 ; 0 err_y];

sGPS     = 0.5  % assume 8.8m/s2 as maximum acceleration, forcing the vehicle
sCourse  = 0.5 % assume 0.1rad/s as maximum turn rate for the vehicle
sVelocity= 3 % assume 8.8m/s2 as maximum acceleration, forcing the vehicle
sYaw     = 0.1 % assume 1.0rad/s2 as the maximum turn rate acceleration for the vehicle

%Q = diag([sGPS^2, sGPS^2, sCourse^2, sVelocity^2, sYaw^2]);
Q = diag([ 0.4*dt 0.4*dt 0.6*dt 3*dt 0.1*dt ]).^2;   


fileID = fopen(filename,'r');
A = fscanf(fileID, data_format);
A = A';
detections = reshape(A, [data_size,size(A,2)/data_size]).';
    

n_measures = 20;

j = 1;
% state [x y yaw speed yawrate]
x = [detections(1,3), detections(1,4),-pi/2,0,0]';
predictions = [];
zetas = [];
scaling_factor = 200000000
start_t = round((detections(1,1)*1e9+detections(1,2))/scaling_factor)
end_t = round((detections(n_measures,1)*1e9+detections(n_measures,2))/scaling_factor)
yaw_est = []
for i=start_t:end_t
    i;
    [x, P] = predict(x, dt,P,Q);
    z = [detections(j,3), detections(j,4)];
    if  j< size(detections, 1) && round((detections(j,1)*1e9+detections(j,2))/scaling_factor) == i
        H = [1.0 0.0 0.0 0.0 0.0; 
             0.0 1.0 0.0 0.0 0.0];
        j = j+1; 
    else
        H = [0.0 0.0 0.0 0.0 0.0; 
             0.0 0.0 0.0 0.0 0.0];
    end
    [x, P] = update(x, P, z, R, H, n_state);
    x
    predictions = [predictions;x'];
    
    if  j< size(detections, 1) && round((detections(j-1,1)*1e9+detections(j-1,2))/scaling_factor) == i
        yaw_est = [yaw_est, x(3)];
        x(3)
    end
    
    zetas = [zetas;z];
end


%compute_angle([2,1], [5,2], [6,4])

yaw_real = [0]
%for i=2:size(detections,1)-1
for i=2:n_measures
    yaw_real = [yaw_real, compute_angle([detections(i-1,3), detections(i-1,4)],[detections(i,3), detections(i,4)] , [detections(i+1,3), detections(i+1,4)])];
end

figure(1);
plot(predictions(:,1), predictions(:,2),'.');
hold on
plot(detections(1:n_measures,3), detections(1:n_measures,4));

% for i=1:size(predictions,1)
%     hold on
%     drawArrow([predictions(i,1),predictions(i,3)], [predictions(i,2),predictions(i,4)]);
% end

plot_zs(predictions, 'blue');


figure(2)

for i=1:n_state
    subplot(n_state,1,i)
    if i ~= 3
        plot(predictions(:,i), 'r');
        hold on
        if i<3
            plot(zetas(:,i), 'b');
        end
    else
        yaw_real
        plot(yaw_est, 'r');
        hold on
        plot(yaw_real, 'b');
    end
    
    

end

function [x_1, P_1] = update(x, P, z, R, H, n_state)
%% kalman filter update

y = z' - H*x;
s = H*P*H' + R;
k = P*H'/s;
x_1 = x + k*y;
P_1 = (eye(n_state)-k*H)*P;

end




function [ ] = plot_zs( z_list, col )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    xs = z_list(:,1);
    ys = z_list(:,2);
    yaws = z_list(:,3);
    u = cos(yaws).*0.1;
    v = sin(yaws).*0.1;
    q_plot = quiver(xs, ys, u, v);
    q_plot.Color = col;
    q_plot.AutoScaleFactor = 0.1;
    %plot(xs, ys, '.')
    
    %set(gca,'XLim',[-36 -32])
    %set(gca,'YLim',[-11.5 -7.5])
end

function [ yaw ] = compute_angle(x1,x2,x3)

m1 = (x1(2)-x2(2)) / (x1(1)-x2(1));
m2 = (x2(2)-x3(2)) / (x2(1)-x3(1));
q1 = (x1(1)*x2(2)-x2(1)*x1(2)) / (x1(1)-x2(1));
q2 = (x2(1)*x3(2)-x3(1)*x2(2)) / (x2(1)-x3(1));

p2p3 = sqrt((x3(1)-x2(1))^2 + (x3(2)-x2(2))^2);
p3p4 = abs(m1*x3(1) - x3(2)+ q1) / sqrt(m1^2+1);
p4p2 = sqrt(p2p3^2 - p3p4^2);

cos_theta = p4p2/p2p3;
sin_theta = p3p4/p2p3;

acos_theta = acos(cos_theta);
asin_theta = asin(sin_theta);

yaw = 0;
for i=1:size(acos_theta,2)
    for j=1:size(asin_theta,2)
        if round(acos_theta(1,i),10) == round(asin_theta(1,j),10)
            yaw =  pi-acos_theta(1,i);
        end
    end
end

if yaw == 0
    acos_theta
    asin_theta
end

end


