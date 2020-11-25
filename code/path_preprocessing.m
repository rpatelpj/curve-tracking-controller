%% Path Preprocessing
% Raj Patel
% Use to update y, dy, ddy, dist_sq, ddist_sq in 'path_following_controller.m'
% and update f in 'main.m'

syms x_R y_R x;
f = @(x) 0.01.*x.^5 - 0.27.*x.^3 + 0.14.*x.^2 + 1.2.*x - 3;
dist = @(x_R, y_R, x) sqrt((x_R - x)^2 + (y_R - f(x))^2);

y = coeffs(f(x), x, 'All')
dy = coeffs(diff(f(x)), x, 'All')
ddy = coeffs(diff(diff(f(x))), x, 'All')
dist_sq = coeffs(dist(x_R, y_R, x).^2, x, 'All')
ddist_sq = coeffs(diff(dist(x_R, y_R, x).^2), x, 'All')