function f = minimum_curvature(p, div_num)

r = return_curvature(p, div_num);
f = sum(r(:, 4).^2);

end