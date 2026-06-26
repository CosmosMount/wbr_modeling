clear; clc;

% ========================================
%  § 数据导入
% ========================================
%  [l,    thetal0,             dl,                  Il]
leg_data = [
    0.13, 1.39493988, 0.11163274, 0.02054335;
    0.14, 1.33429916, 0.11314717, 0.02110900;
    0.15, 1.27783876, 0.11475152, 0.02171655;
    0.16, 1.22487660, 0.11644207, 0.02236600;
    0.17, 1.17489604, 0.11821513, 0.02305735;
    0.18, 1.12749570, 0.12006704, 0.02379060;
    0.19, 1.08235638, 0.12199420, 0.02456575;
    0.20, 1.03921863, 0.12399312, 0.02538280;
    0.21, 0.99786726, 0.12606037, 0.02624175;
    0.22, 0.95812029, 0.12819266, 0.02714260;
    0.23, 0.91982105, 0.13038678, 0.02808535;
    0.24, 0.88283230, 0.13263968, 0.02907000;
    0.25, 0.84703182, 0.13494840, 0.03009655;
    0.26, 0.81230904, 0.13731014, 0.03116500;
    0.27, 0.77856240, 0.13972220, 0.03227535;
    0.28, 0.74569719, 0.14218202, 0.03342760;
    0.29, 0.71362373, 0.14468717, 0.03462175;
    0.30, 0.68225578, 0.14723533, 0.03585780;
    0.31, 0.65150908, 0.14982431, 0.03713575;
    0.32, 0.62129983, 0.15245203, 0.03845560;
    0.33, 0.59154322, 0.15511652, 0.03981735;
    0.34, 0.56215158, 0.15781592, 0.04122100;
    0.35, 0.53303237, 0.16054847, 0.04266655;
    0.36, 0.50408553, 0.16331249, 0.04415400;
    0.37, 0.47519996, 0.16610643, 0.04568335;
    0.38, 0.44624878, 0.16892880, 0.04725460;
    0.39, 0.41708235, 0.17177819, 0.04886775;
    0.40, 0.38751791, 0.17465329, 0.05052280;
];

% Q: [x, dx, phi, dphi, thetall, dthetall, thetalr, dthetalr, thetab, dthetab]
Q = diag([100, 1, 4000, 1, 1000, 10, 1000, 10, 40000, 1]);
% R: [Twl, Twr, Tll, Tlr]
R = diag([10, 10, 1, 1]);

num_legs = size(leg_data, 1);
num_samples = num_legs * num_legs;
k_samples = zeros(4, 10, num_samples);
eq_samples = zeros(2, num_samples);

L_vals = leg_data(:, 1);
R_vals = leg_data(:, 1);

fprintf('Fitting LQR gains for %d samples (dual model)...\n\n', num_samples);
sample_idx = 1;
for i = 1:num_legs
    for j = 1:num_legs
        l_l      = leg_data(i, 1);
        thetall0 = leg_data(i, 2);
        dll      = leg_data(i, 3);
        Ill      = leg_data(i, 4);

        l_r      = leg_data(j, 1);
        thetalr0 = leg_data(j, 2);
        dlr      = leg_data(j, 3);
        Ilr      = leg_data(j, 4);

        [A, B, thetall_eq, thetalr_eq] = matrices(l_l, l_r, thetall0, thetalr0, dll, dlr, Ill, Ilr);
        k_samples(:, :, sample_idx) = -lqr(A, B, Q, R);
        eq_samples(:, sample_idx) = [thetall_eq, thetalr_eq];
        sample_idx = sample_idx + 1;
    end
end

% ========================================
%  § 最小二乘法双参数多项式拟合
% ========================================
[L_grid, R_grid] = meshgrid(L_vals, R_vals);
L_grid = L_grid';
R_grid = R_grid';
L_vec = L_grid(:);
R_vec = R_grid(:);

X = [ones(num_samples, 1), L_vec, R_vec, L_vec.^2, L_vec.*R_vec, R_vec.^2];
eq_coeffs_save = (X \ eq_samples.').';

k_coeffs_save = zeros(4, 10, 6);
for i = 1:4
    for j = 1:10
        k_coeffs_save(i, j, :) = X \ squeeze(k_samples(i, j, :));
    end
end

% ========================================
%  § 格式化输出
% ========================================
fprintf('\t/* dual model: Q = [%.f, %.f, %.f, %.f, %.f, %.f, %.f, %.f, %.f, %.f] */\n', ...
    Q(1,1), Q(2,2), Q(3,3), Q(4,4), Q(5,5), Q(6,6), Q(7,7), Q(8,8), Q(9,9), Q(10,10));
fprintf('\t/* R = [%.f, %.f, %.f, %.f] */\n', R(1,1), R(2,2), R(3,3), R(4,4));
fprintf('\t/* K(L, R) = a1 + a2*L + a3*R + a4*L^2 + a5*L*R + a6*R^2 */\n');
for i = 1:4
    for j = 1:10
        fprintf('\t{%11.6f,%11.6f,%11.6f,%11.6f,%11.6f,%11.6f},\n', ...
            k_coeffs_save(i, j, 1), k_coeffs_save(i, j, 2), k_coeffs_save(i, j, 3), ...
            k_coeffs_save(i, j, 4), k_coeffs_save(i, j, 5), k_coeffs_save(i, j, 6));
    end
end

fprintf('\n\n');
fprintf('/* thetall_eq(L,R) = a1 + a2*L + a3*R + a4*L^2 + a5*L*R + a6*R^2 */\n');
fprintf('float thetall_eq = {%11.6f,%11.6f,%11.6f,%11.6f,%11.6f,%11.6f},\n', ...
    eq_coeffs_save(1,1), eq_coeffs_save(1,2), eq_coeffs_save(1,3), ...
    eq_coeffs_save(1,4), eq_coeffs_save(1,5), eq_coeffs_save(1,6));
fprintf('/* thetalr_eq(L,R) = a1 + a2*L + a3*R + a4*L^2 + a5*L*R + a6*R^2 */\n');
fprintf('float thetalr_eq = {%11.6f,%11.6f,%11.6f,%11.6f,%11.6f,%11.6f},\n', ...
    eq_coeffs_save(2,1), eq_coeffs_save(2,2), eq_coeffs_save(2,3), ...
    eq_coeffs_save(2,4), eq_coeffs_save(2,5), eq_coeffs_save(2,6));
