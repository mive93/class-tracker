close all
clear all
drawArrow = @(x,y) quiver( x(1),y(1),x(2),y(2),0 )    



%% generate trajectory
t = [0 1 2 3 4 5];
points = [0 0; 1 0; 1 1; 0 1; 0 2; 1 2].*10;
x = points(:,1);
y = points(:,2);
tq = 0:0.07:5;
slope0 = 0;
slopeF = 0;
spline_x =  spline(t,[slope0; x; slopeF])
spline_y =  spline(t,[slope0; y; slopeF])

xq = ppval(spline_x,tq)
yq = ppval(spline_y,tq)    

spline_x_d = fnder(spline_x,1);
spline_y_d = fnder(spline_y,1);
%add noise
x_noise = -0.035+0.07*rand(1,size(xq,2));
y_noise = -0.035+0.07*rand(1,size(xq,2));

xq = xq+x_noise;
yq = yq+y_noise;

dx = ppval(spline_x_d,0:0.005:5)
dy = ppval(spline_y_d,0:0.005:5)

yaw = atan2(dy,dx)

%% kalman filter 

dt = 0.005; % delta t
n_state = 5; %number of states in state vector
P = eye(n_state); % covariance

% err_x = 0.01;
% err_y = 0.01;
% 
% Q = [err_x 0 ; 0 err_y]; %measurement noise
% sGPS     = 0.5*2.8*dt^2;  % assume 8.8m/s2 as maximum acceleration, forcing the vehicle
% sCourse  = 0.9*dt; % assume 0.1rad/s as maximum turn rate for the vehicle
% sVelocity= 2.8*dt; % assume 8.8m/s2 as maximum acceleration, forcing the vehicle
% sYaw     = 0.9*dt; % assume 1.0rad/s2 as the maximum turn rate acceleration for the vehicle
% 
% R = diag([sGPS^2, sGPS^2, sCourse^2, sVelocity^2, sYaw^2]); %uncertainty of state transition (Gaussian)


varGPS = 0.2; % Standard Deviation of GPS Measurement
Q = diag([varGPS^2, varGPS^2]);

sGPS     = 5*dt;
sCourse  = 0*dt;
sVelocity= 0.3*dt;
sYaw     = 100*dt;


R = diag([sGPS^2, sGPS^2, sCourse^2, sVelocity^2, sYaw^2]);

j = 1;
x = [xq(1), yq(1),0,0,0]'; % state [x y yaw speed yawrate]
predictions = [];
zetas = [];

for i=0:dt:5
    [x, P] = predict(x, dt,P,R);
    z = [xq(j), yq(j)];
    if  j< size(xq, 2) && ismembertol(i,tq)
        H = [1.0 0.0 0.0 0.0 0.0; 
             0.0 1.0 0.0 0.0 0.0];
        j = j+1; 
    else
        H = [0.0 0.0 0.0 0.0 0.0; 
             0.0 0.0 0.0 0.0 0.0];
    end
    [x, P] = update(x, P, z, Q, H, n_state);
    predictions = [predictions;x'];
    zetas = [zetas;z];
end

%% plot 
figure(1);
plot(predictions(:,1), predictions(:,2),'b.');
hold on
plot(xq, yq, 'r');
plot_zs(predictions(:,1), predictions(:,2), predictions(:,3), 'blue');

figure(2)

labels = ["x" "y" "yaw" "velocity" "yawrate"]
for i=1:n_state
    subplot(n_state,1,i)
    
    plot(predictions(:,i), 'r');
    title(labels(i))
    hold on
    if i<3
        plot(zetas(:,i), 'b');
    end
    if i==3
        plot(yaw, 'b');        
    end
    xlim([0 size(zetas,1)])
end

function [x_1, P_1] = update(x, P, z, Q, H, n_state)
%% kalman filter update

k = P*H'/(H*P*H' + Q); %kalman gain
x_1 = x + k*(z' - H*x); %update mean, incorporating measurement
P_1 = (eye(n_state)-k*H)*P; %update covariance

end



function [x_1, P_1] = predict_easy(x,dt,P,R)
%% kalman filter predict

x_1 = x;

%state transition (update mean)
% x1 = x0 + cos(yaw)*v*dt
% y1 = y0 + sin(yaw)*v*dt
% yaw1 = yaw0 + yawrate*dt
% v = v
% yawrate = yawrate

x_1(1) = x(1)+cos(x(3))*x(4)*dt;
x_1(2) = x(2)+sin(x(3))*x(4)*dt;
x_1(3) = x(3) + x(5)*dt;
x_1(4) = x(4);
x_1(5) = x(5);


%compute jacobian
a13 = -dt*x_1(4)*sin(x_1(3));
a14 = dt*cos(x_1(3));
a23 = dt*x_1(4)*cos(x_1(3));
a24 = dt*sin(x_1(3));
a35 = dt;

J = [1.0    0.0     a13     a14     0.0;
     0.0    1.0     a23     a24     0.0;
     0.0    0.0     1.0     0.0     a35;
     0.0    0.0     0.0     1.0     0.0;
     0.0    0.0     0.0     0.0     1.0;];
%update probs (update covariance)
P_1 = J*P*J' + R;
end


function [x_1, P_1] = predict(x,dt,P,Q)
%% kalman filter predict

x_1 = x;

%new state
if abs(x(5)) < 0.0001 
    %going straight
    x_1(1) = x(1)+cos(x(3))*x(4)*dt;
    x_1(2) = x(2)+sin(x(3))*x(4)*dt;
    x_1(3) = x(3);
    x_1(4) = x(4);
    x_1(5) = 0.0001;
else
    % otherwise
    x_1(1) = x(1)+ (x(4)/x(5)) * (sin(x(5)*dt+x(3)) - sin(x(3)));
    x_1(2) = x(2)+ (x(4)/x(5)) * ((-cos(x(5)*dt+x(3))) + cos(x(3)));
    x_1(3) = x(3) + x(5)*dt;
    x_1(4) = x(4);
    x_1(5) = x(5);
end


a13 = (x_1(4)/x_1(5)) * (cos(x_1(5)*dt+x_1(3)) - cos(x_1(3)));
a14 = (1.0/x_1(5)) * (sin(x_1(5)*dt+x_1(3)) - sin(x_1(3)));
a15 = (dt*x_1(4)/x_1(5))*cos(x_1(5)*dt+x_1(3)) - (x_1(4)/(x_1(5)^2))*(sin(x_1(5)*dt+x_1(3)) - sin(x_1(3)));
a23 = (x_1(4)/x_1(5)) * (sin(x_1(5)*dt+x_1(3)) - sin(x_1(3)));
a24 = (1.0/x_1(5)) * (-cos(x_1(5)*dt+x_1(3)) + cos(x_1(3)));
a25 = (dt*x_1(4)/x_1(5))*sin(x_1(5)*dt+x_1(3)) - (x_1(4)/(x_1(5)^2))*(-cos(x_1(5)*dt+x_1(3)) + cos(x_1(3)));
J = [   1.0     0.0     a13     a14     a15;
        0.0     1.0     a23     a24     a25;
        0.0     0.0     1.0     0.0     dt;
        0.0     0.0     0.0     1.0     0.0;
        0.0     0.0     0.0     0.0     1.0;];

P_1 = J*P*J' + Q;
x-x_1
end

function [ ] = plot_zs( xs, ys, yaws, col )
    %xs = z_list(:,1);
    %ys = z_list(:,2);
    %yaws = z_list(:,3);
    
    size(xs)
    size(ys)
    size(yaws)
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
            yaw =  acos_theta(1,i);
        end
    end
end

if yaw == 0
    acos_theta
    asin_theta
end

end


