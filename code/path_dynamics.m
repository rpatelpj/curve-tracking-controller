%% Path Dynamics
% Raj Patel
function [err_rho, err_theta, dtheta_o] = path_dynamics(q_R, rho_o)
    x_R = q_R(1);
    y_R = q_R(2);
    theta_R = q_R(3);

    % Choose Path Parameters (use 'path_preprocessing.m')
    y = [1/100, 0, -27/100, 7/50, 6/5, -3];
    dy = [1/20, 0, -81/100, 7/25, 6/5];
    ddy = [1/5, 0, -81/50, 7/25];
    dist_sq = [1/10000, 0, -27/5000, 7/2500, 969/10000, - y_R/50 - 339/2500, -1571/2500, (27*y_R)/50 + 489/250, 8/5 - (7*y_R)/25, - 2*x_R - (12*y_R)/5 - 36/5, (y_R + 3)^2 + x_R^2];
    ddist_sq = [1/1000, 0, -27/625, 49/2500, 2907/5000, - y_R/10 - 339/500, -1571/625, (81*y_R)/50 + 1467/250, 16/5 - (14*y_R)/25, - 2*x_R - (12*y_R)/5 - 36/5];

    dist = @(x) sqrt(polyval(dist_sq, x));
    x_d_extrema = roots(ddist_sq);
    x_d_extrema_real = real(x_d_extrema(abs(imag(x_d_extrema)) < 10.^(-10)));
    [rho, x_d_min_ind] = min(arrayfun(dist, x_d_extrema_real));
    err_rho = rho - rho_o;

    x_d_min = x_d_extrema_real(x_d_min_ind);
    theta_o = atan2(polyval(dy, x_d_min), 1);
    dtheta_o = polyval(ddy, x_d_min)/(1 + polyval(dy, x_d_min).^2);
    err_theta = angdiff(theta_R, theta_o);
end