%MODEL_SINGLE  单腿牛顿-欧拉建模 (05_newton_euler_single)
%  完整非线性方程 + 平衡点代入求 M,D,K,H，生成 matrices.m
clear; clc;
addpath(fileparts(fileparts(mfilename('fullpath'))));

p = params();

%% § 符号定义
syms g real
syms mb ml mw real
syms Ib Iw real
syms db real
syms dll dlr Ill Ilr real
syms thetall0 thetalr0 real   %#ok<NASGU>
syms ll lr real
syms Rw real

syms x thetal thetab real
syms dx dthetal dthetab real
syms ddx ddthetal ddthetab real

syms Tw Tl real

l   = ll;
l_w = ll;
l_b = dll;
I_l = Ill;

%% § 运动学 (单轮滚动约束)
ddthetaw = ddx / Rw;

a_l_v = -l_w*sin(thetal)*ddthetal - l_w*cos(thetal)*dthetal^2;
a_b_v = -l*sin(thetal)*ddthetal   - l*cos(thetal)*dthetal^2;
a_b_h = ddx + l*cos(thetal)*ddthetal - l*sin(thetal)*dthetal^2;

%% § 动力学方程 (05 文档, 消力后的完整非线性式)
% 水平运动 (式 155-157)
eqn1 = (mw + ml + mb + Iw/Rw^2)*ddx ...
     + (ml + mb)*(l_w*cos(thetal)*ddthetal - l_w*sin(thetal)*dthetal^2) ...
     + mb*(l_b*cos(thetal)*ddthetal - l_b*sin(thetal)*dthetal^2) ...
     - Tw/Rw;

% 机体俯仰 (式 170-172)
eqn2 = Ib*ddthetab + Tl ...
     - mb*(a_l_v + g)*db*sin(thetab) ...
     + mb*a_b_h*db*cos(thetab);

% 腿部转动 (式 197-202)
F_wh = Tw/Rw - Iw*ddthetaw/Rw - mw*ddx;

eqn3 = I_l*ddthetal - Tl + Tw ...
     + mb*l*sin(thetal)*(-l*sin(thetal)*ddthetal - l*cos(thetal)*dthetal^2 + g) ...
     + ml*l_w*sin(thetal)*(-l_w*sin(thetal)*ddthetal - l_w*cos(thetal)*dthetal^2 + g) ...
     - l_w*cos(thetal)*F_wh ...
     - mb*l_w*cos(thetal)*(ddx + l*cos(thetal)*ddthetal - l*sin(thetal)*dthetal^2);

%% § 平衡点 (无质心偏置, 竖直平衡)
thetal_eq = sym(0);
thetab_eq = sym(0);

%% § 代入物理常数
params_subs = {
    g,  p.g;
    Rw, p.Rw;
    mb, p.mb;
    Ib, p.Ib;
    db, p.db;
    mw, p.mw;
    Iw, p.Iw;
    ml, p.ml;
};

thetal_eq = simplify(subs(thetal_eq, params_subs(:,1), params_subs(:,2)));
eqns_vec = simplify(subs([eqn1; eqn2; eqn3], params_subs(:,1), params_subs(:,2)));

u   = [Tw; Tl];
q   = [x; thetal; thetab];
dq  = [dx; dthetal; dthetab];
ddq = [ddx; ddthetal; ddthetab];

%% § 提取 M, D, K, H
fprintf('Computing Jacobians for single ...\n');
M_sym = jacobian(eqns_vec, ddq);
D_sym = jacobian(eqns_vec, dq);
K_sym = jacobian(eqns_vec, q);
H_sym = jacobian(eqns_vec, u);

M_param = simplify(subs(M_sym, params_subs(:,1), params_subs(:,2)));
D_param = simplify(subs(D_sym, params_subs(:,1), params_subs(:,2)));
K_param = simplify(subs(K_sym, params_subs(:,1), params_subs(:,2)));
H_param = simplify(subs(H_sym, params_subs(:,1), params_subs(:,2)));

%% § 平衡点代入
eq_subs = {
    thetal, thetal_eq;
    thetab, thetab_eq;
    dthetal, 0; dthetab, 0;
    x, 0; dx, 0;
    ddx, 0; ddthetal, 0; ddthetab, 0;
    Tw, 0; Tl, 0;
};

M_eq = simplify(subs(M_param, eq_subs(:,1), eq_subs(:,2)));
D_eq = simplify(subs(D_param, eq_subs(:,1), eq_subs(:,2)));
K_eq = simplify(subs(K_param, eq_subs(:,1), eq_subs(:,2)));
H_eq = simplify(subs(H_param, eq_subs(:,1), eq_subs(:,2)));

%% § 状态空间 A, B
n = 6; m_ctrl = 2;
A_num = sym(zeros(n, n));
B_num = sym(zeros(n, m_ctrl));

A_num(1,2) = 1;
A_num(3,4) = 1;
A_num(5,6) = 1;

M_inv = inv(M_eq);
A_num(2:2:6, 1:2:5)  = -M_inv * K_eq;
A_num(2:2:6, 2:2:6) = -M_inv * D_eq;
B_num(2:2:6, :)      = -M_inv * H_eq;

thetall_eq_sym = thetal_eq;
thetalr_eq_sym = thetal_eq;

fprintf('Generating single/matrices.m ...\n');
matlabFunction(A_num, B_num, thetall_eq_sym, thetalr_eq_sym, ...
    'File', 'matrices', ...
    'Vars', {ll, lr, thetall0, thetalr0, dll, dlr, Ill, Ilr}, ...
    'Outputs', {'A', 'B', 'thetall_eq', 'thetalr_eq'});
fprintf('Done.\n');
