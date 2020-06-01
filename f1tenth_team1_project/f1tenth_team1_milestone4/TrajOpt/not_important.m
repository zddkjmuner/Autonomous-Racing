wp = load('wp.mat');
seg_idx = linspace(1, 7280, 364);
% plot(wp.wp(range, 1), wp.wp(range, 2));
f = fit(wp.wp(range, 1), wp.wp(range, 2), 'spline');
plot(f,wp.wp(range, 1), wp.wp(range, 2));

for i = 1:364;
    batch_x = wp.wp(seg_idx(i):seg_idx(i+1)-1, 1);
    batch_x = wp.wp(seg_idx(i):seg_idx(i+1)-1, 1);
end