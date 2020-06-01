function [path_desc] = return_curvature(p, div_num, velocity)
%% initialize main loop
k = 1;
path_desc= [];
stop = 0;
num_points = size(p, 1);

while(~stop)
%stop condition (when all points of path are used)
if((k+div_num) > num_points)
    indices = k:size(p, 1);
    stop = 1;
else
    indices = k:k+div_num;
    k = k+div_num+1;
end

% if(numel(indices) < 3)
%    break; 
%    stop_here = 1;
% end
%% split path into batches and decribe as curvilinear distance s and curvature K
batch_x = p(indices, 1); batch_y = p(indices, 2);
batch_vel = velocity(indices);
%add distance between each point in arc to get total curvilinear distance
s = sqrt((batch_x(2:end) - batch_x(1:end-1)).^2 + ...
    (batch_y(2:end) - batch_y(1:end-1)).^2);
t = sum(s./batch_vel(2:end));

% plotcircfit(batch_x, batch_y);
mid_idx = floor(numel(indices)/2); 

pt1 = [batch_x(1), batch_y(1)];
pt2 = [batch_x(3), batch_y(3)];
n = null(pt1 - pt2);
n = n/norm(n);
path_desc = [path_desc; batch_x(1), batch_y(1), n(1), n(2), t, t, batch_x(mid_idx), batch_y(mid_idx)];
end

% quiver(path_desc(:, 1), path_desc(:, 2), path_desc(:, 3), path_desc(:, 4));
% hold on
% plot(path_desc(:, 1), path_desc(:, 2), 'rx')