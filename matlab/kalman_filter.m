close all
clear all
drawArrow = @(x,y) quiver( x(1),y(1),x(2),y(2),0 )    

%filename = 'test.txt'
%data_format = '%d\t%d\t%d\t%d\t%d\t%d\t%d\n'
%data_size = 7

filename = 'gt_000002.txt'
data_format = '%d\t%d\t%f\t%f\n'
data_size = 4

delta_t = 0.45/2; %seconds
err_x = 0.005;
err_y = 0.005;
nx = 0.03;
ny = 0.03;

P = eye(4);
F = eye(4);
F(1,3) = delta_t;
F(2,4) = delta_t;
H = [1 0 0 0 ; 0 1 0 0];
R = [err_x 0 ; 0 err_y];
Q = [   delta_t^4/4*nx  0               delta_t^3/2*nx	0;
        0               delta_t^4/4*ny  0               delta_t^3/2*ny;
        delta_t^3/2*nx  0               delta_t^2^nx    0;
        0               delta_t^3/2*ny  0               delta_t^2*ny    ];


    
fileID = fopen(filename,'r');
A = fscanf(fileID, data_format);
A = A';
detections = reshape(A, [data_size,size(A,2)/data_size]).';
    
    
j = 1;
x = [detections(1,3), detections(1,4),0,0]';
predictions = [];
scaling_factor = 200000000
start_t = round((detections(1,1)*1e9+detections(1,2))/scaling_factor)
end_t = round((detections(150,1)*1e9+detections(150,2))/scaling_factor)
for i=start_t:end_t
 
    i
    [x, P] = predict(x,F,P,Q);
    z = [detections(j,3), detections(j,4)];
    if  j< size(detections, 1) && round((detections(j,1)*1e9+detections(j,2))/scaling_factor) == i
        %[x, P] = predict(x,F,P,Q);
        H = [1 0 0 0 ; 0 1 0 0];
        j = j+1
    else
        H = [0 0 0 0 ; 0 0 0 0];
    end
    [x, P] = update(x, P, z, R, H);
    predictions = [predictions;x'];
end
    
figure(1);
plot(predictions(:,1), predictions(:,2),'.');
hold on
plot(detections(1:150,3), detections(1:150,4));
for i=1:size(predictions,1)
    hold on
    drawArrow([predictions(i,1),predictions(i,3)], [predictions(i,2),predictions(i,4)]);
end

    
% j = 1;
% k = 0
% x = [detections(1,4), detections(1,5),0,0]';
% predictions = [];
% for i=1:450
%     [x, P] = predict(x,F,P,Q);
%     if  j< size(detections, 1) && detections(j,3) == i
%         k = k+1
%         H = [1 0 0 0 ; 0 1 0 0];
%         if mod(k,20) == 0 
%             z = [detections(j,4), detections(j,5)];
%         end
%         j = j+1;
%     else
%         H = [0 0 0 0 ; 0 0 0 0];
%     end
%     
%     [x, P] = update(x, P, z, R, H);
%     predictions = [predictions;x'];
% end
%     
% figure(1);
% plot(predictions(:,1), predictions(:,2),'.');
% hold on
% plot(detections(:,4), detections(:,5));
% for i=1:size(predictions,1)
%     hold on
%     drawArrow([predictions(i,1),predictions(i,3)], [predictions(i,2),predictions(i,4)]);
% end



function [x_1, P_1] = update(x, P, z, R, H)

y = z' - H*x;
s = H*P*H' + R;
k = P*H'/s;
x_1 = x + k*y;
P_1 = (eye(4)-k*H)*P;

end

function [x_1, P_1] = predict(x,F,P,Q)

x_1 = F*x;
P_1 = F*P*F' + Q;

end




