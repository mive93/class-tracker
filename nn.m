clear all 
close all

data_size = 4;
filename = 'prova_pixel.txt';
data_format = '%d\t%d\t%d\t%d\n';

detections = readAndPlotData(filename, data_format, data_size);

prev = -1;
prev_frame = [];
cur_frame = [];
index = 1;
for i=index:size(detections,1)
    if detections(i,1) ~= prev 
        
        if ~isempty(cur_frame)
            index = i;
            break;
        end
        prev = detections(i,1);
    end
    cur_frame = [ cur_frame; [detections(i,3) detections(i,4)]];
end


ageTresh = -10;
initialAge = -5;

% traj_i = 2;
for j=1:size(cur_frame)
    traj{j} = [cur_frame(j, :), prev];
end
trajAge = zeros(1,size(traj,2))*initialAge;



for i=index:size(detections,1)
    if detections(i,1) ~= prev  
        
        prev_frame;
        cur_frame;
        
        if ~isempty(prev_frame)
            
            [traj, trajAge] = deleteOldTraj(traj, trajAge, ageTresh);
            
            prev_traj=[];
            for j=1:size(traj,2)
                prev_traj = [prev_traj;traj{j}(end,1:2)];
            end
            
            [res, D] = knnsearch(prev_traj,cur_frame);
            d = ones(size(traj,2),1)*50;
                       
            new_traj = zeros(size(prev_traj));
            
            
            trajAge = trajAge-1;
            resD = [res, D, [1:size(res,1)]'];
            resD = sortrows(resD);
            used = zeros(1,size(res,1));
            for j=1:size(resD,1)
                if resD(j,1) <= size(traj,2)
                    if resD(j,2) <= d(resD(j,1))
                        traj{resD(j,1)} = [traj{resD(j,1)}; [cur_frame(resD(j,3), :),prev]] ;
                        d(resD(j,1)) = resD(j,2);
                        used(j) = 1;
                        trajAge(resD(j,1)) = trajAge(resD(j,1))+2;
                    end
                end
            end
            
            for j=1:size(resD,1)
                if used(j) == 0
                    traj{size(traj,2)+1} = [cur_frame(resD(j,3), :),prev];
                    trajAge = [trajAge, initialAge];
                end
            end
            
%             for j=1:size(res)
%                 if res(j) <= size(traj,2)
%                     if D(j) <= d(res(j))
%                         new_traj(res(j),:) = cur_frame(j, :);
%                         d(res(j)) = D(j) ;
%                     end
%                 end
%             end
%             for j=1:size(new_traj)
%                 if new_traj(j,:) > 0
%                     traj{j} = [traj{j}; [new_traj(j, :),prev]];                    
%                     trajAge(j) = trajAge(j)+1;
%                 else
%                     trajAge(j) = trajAge(j)-1;
%                 end 
%             end
            
        end
        
        prev_frame = cur_frame;
        cur_frame = [];
        prev = detections(i,1);
    end
    cur_frame = [ cur_frame; [detections(i,3) detections(i,4)]];
end


for j=1:size(traj,2)
    %traj{j} = rmoutliers(traj{j})
    figure;
    plot(traj{j}(:,1), traj{j}(:,2), 'r');
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
    
    figure;
    frameid = arrayfun(@num2str, detections(:,1), 'uniform', 0);
    plot(detections(:,3), detections(:,4), '.');
    text(detections(:,3), detections(:,4),frameid,'VerticalAlignment','bottom','HorizontalAlignment','right')
end


function [cleanTraj, cleanAge] = deleteOldTraj(traj, trajAge, ageTresh)
    j=1;
    cleanAge = [];
    cleanTraj = {};
    for i=1:size(traj,2)
        if trajAge(i) > ageTresh
            cleanTraj{j} = traj{i};
            cleanAge = [cleanAge, trajAge(i)];
            j = j+1;
        end
    end
end