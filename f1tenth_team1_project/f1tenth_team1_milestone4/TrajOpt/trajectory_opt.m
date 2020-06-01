wp = load('wp.mat');
wp.wp = wp.wp(1:7300, :);
num_points = size(wp.wp, 1);
div_num = 300;

% %discretize path and get curvature, curvilinear distance, normal at start point for each segment 
velo = velocity_profile(wp.wp(:,1), wp.wp(:,2));
path_desc = return_curvature(wp.wp, div_num, velo);

%initialize CMAES
dim = size(path_desc, 1);
gen_size = 50;
alphas = zeros(dim, gen_size);

mu = zeros(dim, 1);
covar = eye(dim);
stop = 0;
iter = 0;
while(iter<2000)
    score = zeros(gen_size, 1);
for i = 1:gen_size
    rng shuffle
    alphas(:, i) = 0.1*covar*(rand(dim, 1) - 0.5) + mu;
    path = path_desc(:, 1:2) + repmat(alphas(:,i), 1, 2).*path_desc(:, 3:4);
    x_samples = [];
    y_samples = [];
    for j = 1:2:dim
        if(j == dim)
            ptx = [path(j, 1); path(1:2, 1)];
            pty = [path(j, 2); path(1:2, 2)];
        elseif(j==dim-1)
            ptx = [path(j:dim, 1); path(1, 1)];
            pty = [path(j:dim, 2); path(1, 2)];
        else
            ptx = path(j:j+2, 1);
            pty = path(j:j+2, 2);
        end
        [val, gof] = fit(ptx, pty, 'spline');
        samples = [linspace(ptx(1), ptx(2), 50),linspace(ptx(2)+0.005, ptx(3), 50)]';
        num_samples = numel(samples);
        x_samples = [x_samples; samples];
        y_samples = [y_samples; val(samples)];
        
    end
    vel = velocity_profile(x_samples, y_samples);
    r = return_curvature([x_samples, y_samples], num_samples, vel);
    score(i) = score(i) + sum(r(:, 6).^2, 'all');
end
best_scores = min(maxk(score, 5));
best_gen_idx = find(score >= best_scores);
new_sd = sqrt((1/numel(best_gen_idx))*sum((repmat(mu, 1, numel(best_gen_idx)) - alphas(:, best_gen_idx)).^2, 2));

covar = zeros(dim, dim);
for m = 1:dim
    mu_m = mu(m);
    for n = 1:dim
        new_sd = sqrt((1/numel(best_gen_idx))*sum((repmat(mu_m, 1, numel(best_gen_idx)) - alphas(n, best_gen_idx)).^2, 2));
        covar(m, n) = new_sd;
    end
end
mu = sum(alphas(:, best_gen_idx), 2)/numel(best_gen_idx);

iter = iter+1
end

final_best_score = max(score);
final_best_idx = find(score == final_best_score);
best_alpha = alphas(:, final_best_idx);
best_path = path_desc(:, 1:2) + repmat(best_alpha, 1, 2).*path_desc(:, 3:4);

plot(best_path(:, 1), best_path(:,2), 'b-');
hold on;
plot(path_desc(:, 1), path_desc(:,2), 'r-')
