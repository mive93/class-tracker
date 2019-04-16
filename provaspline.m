clear all
close all

t = [0 1 2 3 4 5];
points = [0 0; 1 0; 1 1; 0 1; 0 2; 1 2];
x = points(:,1);
y = points(:,2);

tq = 0:0.1:5;
slope0 = 0;
slopeF = 0;
xq = spline(t,[slope0; x; slopeF],tq);
yq = spline(t,[slope0; y; slopeF],tq);

x_noise = -0.035+0.07*rand(1,size(xq,2))
y_noise = -0.035+0.07*rand(1,size(xq,2))

xqn = xq+x_noise
yqn = yq+y_noise



% figure()
% plot(xqn,yqn, '.');


% 
% figure;
% plot(t,y,'o',tq,yq,':.');
% axis([-0.5 5.5 -0.5 2.5]);
% title('y vs. t');
% 
% figure()
% plot(xq,yq);
% axis([-0.5 1.5 -0.5 2.5]);
 
% figure()
% x = linspace(0,2*pi,100);
% noisy_y = cos(x) + .2*(rand(size(x))-.5);
% plot(x,noisy_y,'x')
% axis([-1 7 -1.2 1.2])
% 
% figure()
% spl2 = spap2(4, 4, x, noisy_y);
% fnplt(spl2,'b',2);
% axis([-1 7 -1.2 1.2])
% hold on
% plot(x,noisy_y,'x')
% hold off
% 
% figure()
% x = linspace(0,10,101);
% y = exp(x);
% sp0 = spap2( augknt(0:2:10,4), 4, x, y );
% sp1 = spap2( newknt(sp0), 4, x, y );
% plot(x,y-fnval(sp0,x),'k','LineWidth',2)





