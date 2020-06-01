%% velocity profile given a path
function final_vel = velocity_profile(x, y)
wp.wp = [x, y];
num_points = size(wp.wp, 1);
div_num = 200;

%% car properties
g = 9.8; % gravity
mu = 0.523; % friction coefficient
max_acc = 7.51;
max_decc = 8.26;


%% initialize main loop
k = 1;
path= [];
indices = [];
stop = 0;
while(~stop)
%stop condition (when all points of path are used)
if((k+div_num) > num_points)
    indices = k:size(wp.wp, 1);
    stop = 1;
else
    indices = k:k+div_num;
    k = k+div_num+1;
end
%% split path into batches and decribe as curvilinear distance s and curvature K
batch_x = wp.wp(indices, 1); batch_y = wp.wp(indices, 2);

%add distance between each point in arc to get total curvilinear distance
s = sqrt(sum((batch_x(2:end) - batch_x(1:end-1)).^2 + ...
    (batch_y(2:end) - batch_y(1:end-1)).^2));

[~, ~, Rfit] = circfit(batch_x, batch_y );
% plotcircfit(batch_x, batch_y);
path_desc = repmat([s, Rfit], numel(indices), 1);
path = [path; batch_x, batch_y, path_desc];

end

%% velocity profile generation
init_vel = sqrt(mu*g*abs(path(:,4)));

% plot(1:length(init_vel),init_vel);

%forward pass - considering acceleration limits
fwd_vel = init_vel;
for i = 2:num_points
    del_s = sqrt((path(i,1) - path(i-1, 1))^2 + (path(i,2)-path(i-1,2))^2);
    temp = sqrt(fwd_vel(i-1)^2 + 2*max_acc*del_s);
    if(temp < init_vel(i))
       fwd_vel(i) = temp;
    else
       fwd_vel(i) = init_vel(i);
    end
end

%backward pass - considering decceleration limits
bwd_vel = fwd_vel;
flag = 0;
for i = num_points-1:-1:1
    del_s = sqrt((path(i,1) - path(i+1, 1))^2 + (path(i,2)-path(i+1,2))^2);
    
    if(fwd_vel(i+1)<fwd_vel(i) || flag == 1)
        temp = sqrt(bwd_vel(i+1)^2 + 2*max_decc*del_s);
        if(temp<fwd_vel(i))
            bwd_vel(i) = temp;
            flag = 1;
        else
            flag = 0;
        end
    end
    if(fwd_vel(i+1)>= fwd_vel(i) && flag == 0)
       bwd_vel(i) = fwd_vel(i+1);
    end
end

%plot init velcity vs final velocity
% figure; plot(1:num_points, init_vel);
% hold on
% plot(1:num_points,bwd_vel, 'r-');
% hold off

max_vel = max(bwd_vel);
final_vel = (0.7/max_vel)*bwd_vel + 3.7;
% figure;plot(1:num_points, final_vel);

end