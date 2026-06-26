%DUAL_OFFSET_MODEL  双腿质心偏移建模 (07_newton_euler_dual_with_offset)
%  完整非线性方程 + 平衡点代入求 M,D,K,H，生成 matrices.m
clear; clc;
addpath(fileparts(fileparts(mfilename('fullpath'))));

p = params();

%% § 符号定义
syms g real
syms mb ml mw real
syms Ib Iphi Iw real
syms db thetab0 real
syms dll dlr thetall0 thetalr0 real
syms Ill Ilr real
syms Rb Rw real

syms x phi thetall thetalr thetab real
syms dx dphi dthetall dthetalr dthetab real
syms ddx ddphi ddthetall ddthetalr ddthetab real
syms ll lr real

syms Twl Twr Tll Tlr real

%% § 运动学约束 (滚动 + yaw 耦合)
a_l_h = ll*cos(thetall)*ddthetall - ll*sin(thetall)*dthetall^2 ...
      + lr*cos(thetalr)*ddthetalr - lr*sin(thetalr)*dthetalr^2;

ddthetawl = (ddx - Rb*ddphi - a_l_h/2) / Rw;
ddthetawr = (ddx + Rb*ddphi - a_l_h/2) / Rw;

a_ll_v = -ll*sin(thetall)*ddthetall - ll*cos(thetall)*dthetall^2;
a_lr_v = -lr*sin(thetalr)*ddthetalr - lr*cos(thetalr)*dthetalr^2;

%% § 动力学方程 (07 文档)
% 水平动量
eqn1 = (Twl - Iw*ddthetawl + Twr - Iw*ddthetawr) ...
     - ((mb/2 + ml + mw)*Rw*(ddthetawl + ddthetawr) + (mb/2 + ml)*a_l_h)*Rw;

% yaw
eqn2 = Iphi*ddphi - ((Twr - Iw*ddthetawr) - (Twl - Iw*ddthetawl))*Rb/Rw;

% 左腿
eqn3 = Tll - Twl - ml*g*dll*sin(thetall + thetall0) ...
     - ((Twl - Iw*ddthetawl)/Rw - mw*Rw*ddthetawl)*ll*cos(thetall) ...
     + ((mb/2 + ml)*(g + (a_ll_v + a_lr_v)/2) - mw*g)*ll*sin(thetall) ...
     - Ill*ddthetall;

% 右腿
eqn4 = Tlr - Twr - ml*g*dlr*sin(thetalr + thetalr0) ...
     - ((Twr - Iw*ddthetawr)/Rw - mw*Rw*ddthetawr)*lr*cos(thetalr) ...
     + ((mb/2 + ml)*(g + (a_ll_v + a_lr_v)/2) - mw*g)*lr*sin(thetalr) ...
     - Ilr*ddthetalr;

% 机体俯仰
eqn5 = (-Tll - Tlr - mb*g*db*cos(thetab + thetab0)) - Ib*ddthetab;

%% § 平衡点 (07 文档)
thetall_eq = atan2(-ml*dll*sin(thetall0), (ml + mb/2)*ll + ml*dll*cos(thetall0));
thetalr_eq = atan2(-ml*dlr*sin(thetalr0), (ml + mb/2)*lr + ml*dlr*cos(thetalr0));
thetab_eq  = pi/4 - p.thetab0/2;

%% § 代入物理常数
params_subs = {
    g,     p.g;
    Rb,    p.Rb;
    Rw,    p.Rw;
    mb,    p.mb;
    Ib,    p.Ib;
    db,    p.db;
    Iphi,  p.Iphi;
    thetab0, p.thetab0;
    mw,    p.mw;
    Iw,    p.Iw;
    ml,    p.ml;
};

thetall_eq = simplify(subs(thetall_eq, params_subs(:,1), params_subs(:,2)));
thetalr_eq = simplify(subs(thetalr_eq, params_subs(:,1), params_subs(:,2)));

eqns_vec = simplify(subs([eqn1; eqn2; eqn3; eqn4; eqn5], ...
    params_subs(:,1), params_subs(:,2)));

u   = [Twl; Twr; Tll; Tlr];
q   = [x; phi; thetall; thetalr; thetab];
dq  = [dx; dphi; dthetall; dthetalr; dthetab];
ddq = [ddx; ddphi; ddthetall; ddthetalr; ddthetab];

%% § 提取 M, D, K, H
fprintf('Computing Jacobians for dual_offset ...\n');
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
    thetall, thetall_eq;
    thetalr, thetalr_eq;
    thetab,  thetab_eq;
    dthetall, 0; dthetalr, 0; dthetab, 0;
    phi, 0; dphi, 0;
    x, 0; dx, 0;
    ddx, 0; ddphi, 0; ddthetall, 0; ddthetalr, 0; ddthetab, 0;
    Twl, 0; Twr, 0; Tll, 0; Tlr, 0;
};

M_eq = simplify(subs(M_param, eq_subs(:,1), eq_subs(:,2)));
D_eq = simplify(subs(D_param, eq_subs(:,1), eq_subs(:,2)));
K_eq = simplify(subs(K_param, eq_subs(:,1), eq_subs(:,2)));
H_eq = simplify(subs(H_param, eq_subs(:,1), eq_subs(:,2)));

%% § 状态空间 A, B
n = 10; m_ctrl = 4;
A_num = sym(zeros(n, n));
B_num = sym(zeros(n, m_ctrl));

A_num(1,2) = 1;  A_num(3,4) = 1;
A_num(5,6) = 1;  A_num(7,8) = 1;
A_num(9,10) = 1;

M_inv = inv(M_eq);
A_num(2:2:10, 1:2:9)  = -M_inv * K_eq;
A_num(2:2:10, 2:2:10) = -M_inv * D_eq;
B_num(2:2:10, :)      = -M_inv * H_eq;

fprintf('Generating dual_offset/matrices.m ...\n');
matlabFunction(A_num, B_num, thetall_eq, thetalr_eq, ...
    'File', 'matrices', ...
    'Vars', {ll, lr, thetall0, thetalr0, dll, dlr, Ill, Ilr}, ...
    'Outputs', {'A', 'B', 'thetall_eq', 'thetalr_eq'});
fprintf('Done.\n');
