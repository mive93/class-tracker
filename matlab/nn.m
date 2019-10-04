clear all 
close all

data_size = 4;
filename = '../data/test_ll.txt';
data_format = '%d\t%d\t%f\t%f\n';

addpath('/home/micaela/repos/tkmatlab/src/ekf');

detections = readAndPlotData(filename, data_format, data_size);

cur_frame = [];
ageTresh = -10;
initialAge = -5;

traj = {};
prev=0;

ekfList = {};
zList    = {};
predList = {}; 

dt=0.033;
stateDim = 5;

for i=1:size(detections,1)
    if detections(i,1) ~= prev  
       
        if isempty(traj)
            %% first detection (initialization)
            for j=1:size(cur_frame)
                traj{j} = [cur_frame(j, :), prev];
                ekfList{j} = initilizeEKF(dt, stateDim, cur_frame(j, 1), cur_frame(j, 2));
                predList{j} = [];
                zList{j} = [];
            end
            trajAge = zeros(1,size(traj,2))*initialAge;
        else
            %% delete old trajectories 
            [traj, trajAge, ekfList, zList, predList] = deleteOldTraj(traj, trajAge, ageTresh, ekfList, zList, predList);
            
            %% update trajectories
            prev_traj=[];
            for j=1:size(traj,2)
                prev_traj = [prev_traj;traj{j}(end,1:2)];
            end
            
            [res, D] = knnsearch(prev_traj,cur_frame);
            resD = [res, D, [1:size(res,1)]'];
            resD = sortrows(resD);
            
            used = zeros(1,size(res,1)); 
            d = ones(size(traj,2),1)*50; %distance allowed from prev point
            trajAge = trajAge-1; %aging
            
            newTraj = zeros(size(traj,2),2);
            for j=1:size(resD,1)
                if resD(j,1) <= size(traj,2)
                    if resD(j,2) <= d(resD(j,1))
                        traj{resD(j,1)} = [traj{resD(j,1)}; [cur_frame(resD(j,3), :),prev]] ;
                        newTraj(resD(j,1),:) = cur_frame(resD(j,3),:);
                        d(resD(j,1)) = resD(j,2);
                        trajAge(resD(j,1)) = trajAge(resD(j,1))+2; %aging
                        used(j) = 1;
                    end
                end
            end
            
            
            %% KALMAN
            for j=1:size(newTraj,1)
                ekf = ekfList{j};
                if newTraj(j,:) == 0
                    ekf.H(1,1) = 0;
                    ekf.H(2,2) = 0; 
                else
                    ekf.H(1,1) = 1;
                    ekf.H(2,2) = 1; 
                end
                
                new_z = zeros(stateDim,1);
                new_z(1) = newTraj(j,1); 
                new_z(2) = newTraj(j,2); 

                ekf = EKF_step(ekf, new_z);
                
                predList{j} = [ predList{j}; ekf.xEst'];
                
                if newTraj(j,:) ~= 0
                    zList{j}  = [ zList{j}; new_z'];
                end
                ekfList{j} = ekf;
            end
            
            
            %% add new trajectories
            for j=1:size(resD,1)
                if used(j) == 0
                    traj{size(traj,2)+1} = [cur_frame(resD(j,3), :),prev];
                    ekfList{size(ekfList,2)+1} = initilizeEKF(dt, stateDim, cur_frame(resD(j,3), 1),cur_frame(resD(j,3), 2));
                    predList{size(predList,2)+1} = [];
                    zList{size(zList,2)+1} = [];
                    trajAge = [trajAge, initialAge];
                end
            end
        end
        
        cur_frame = [];
        prev = detections(i,1);
    end
    cur_frame = [ cur_frame; [detections(i,3) detections(i,4)]];
end


for j=1:size(traj,2)
    %traj{j} = rmoutliers(traj{j})
    figure;
    plot(zList{j}(:, 1), zList{j}(:,2), 'r');
    hold on;
    plot(predList{j}(:, 1), predList{j}(:,2), 'g');
    legend('groundtruth','prediction')
%     plot(traj{j}(:,1), traj{j}(:,2), 'r');
end

% traj = rmoutliers(traj)
% figure;
% plot(traj(:,1), traj(:,2), 'r');
% frameid = arrayfun(@num2str, traj(:,3), 'uniform', 0);
% text(traj(:,1), traj(:,2),frameid,'VerticalAlignment','bottom','HorizontalAlignment','right')


function detections = readAndPlotData(filename, data_format, data_size)
    fileID = fopen(filename,'r');
    A = fscanf(fileID, data_format);
    A = A';
    detections = reshape(A, [data_size,size(A,2)/data_size]).';
    
    spheroid = referenceEllipsoid('GRS 80');
    lat0 = 44.655540;
    lon0 = 10.934315;
    for i=1:size(detections,1)
        [detections(i, 3), detections(i, 4), h] = geodetic2enu(detections(i, 3), detections(i, 4),0,lat0,lon0,0,spheroid);
    end
    
    
    figure;
    frameid = arrayfun(@num2str, detections(:,1), 'uniform', 0);
    plot(detections(:,3), detections(:,4), '.');
    text(detections(:,3), detections(:,4),frameid,'VerticalAlignment','bottom','HorizontalAlignment','right')
end

function ekf = initilizeEKF(dt, stateDim, x0, y0)
    ekf = EKF;
    ekf = EKF_init(ekf, dt);
    % Covariance Matrix for motion
    ekf.Q    = (diag([ 1*dt 1*dt 1*dt 3*dt 0.1*dt ]).*1).^2;
    % Covariance Matrix for observation
    ekf.R    = (diag([0.5 0.5 0.1 0.5 0.02]).*1).^2; 
    ekf.H    = zeros(stateDim);
    
    ekf.xEst = [ x0, y0, 0, 0, 0 ]';
end


function [cleanTraj, cleanAge, ekfClean, zClean, predClean] = deleteOldTraj(traj, trajAge, ageTresh, ekfList, zList, predList)
    j=1;
    cleanAge = [];
    cleanTraj = {};
    for i=1:size(traj,2)
        if trajAge(i) > ageTresh
            cleanTraj{j} = traj{i};
            ekfClean{j} = ekfList{i};

            zClean{j} = zList{i};
            predClean{j} = predList{i};
            
            cleanAge = [cleanAge, trajAge(i)];
            j = j+1;
        else
            if size(traj{i},1) > 10
                figure;
                plot(zList{i}(:, 1), zList{i}(:,2), 'b');
                hold on;
                plot(predList{i}(:, 1), predList{i}(:,2), 'g');
%                 plot(traj{i}(:,1), traj{i}(:,2), 'b');
                legend('groundtruth','prediction')
            end
        end
    end
end