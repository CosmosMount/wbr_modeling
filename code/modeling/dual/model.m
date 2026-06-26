%DUAL_MODEL  双腿牛顿-欧拉建模 (06_newton_euler_dual)
%  完整非线性方程 + 平衡点代入求 M,D,K,H，生成 matrices.m
clear; clc;
addpath(fileparts(fileparts(mfilename('fullpath'))));

p = params();

%% § 符号定义
syms g real
syms mb ml mw real
syms Ib Iphi Iw real
syms db real
syms dll dlr Ill Ilr real
syms thetall0 thetalr0 real   %#ok<NASGU>
syms ll lr real
syms Rb Rw real

syms x phi thetall thetalr thetab real
syms dx dphi dthetall dthetalr dthetab real
syms ddx ddphi ddthetall ddthetalr ddthetab real

syms Twl Twr Tll Tlr real

l_l  = ll;  l_r  = lr;
l_wl = ll;  l_wr = lr;
l_bl = dll; l_br = dlr;

%% § 运动学约束 (滚动 + yaw 耦合)
a_l_h = l_l*cos(thetall)*ddthetall - l_l*sin(thetall)*dthetall^2 ...
      + l_r*cos(thetalr)*ddthetalr - l_r*sin(thetalr)*dthetalr^2;

ddthetawl = (ddx - Rb*ddphi - a_l_h/2) / Rw;
ddthetawr = (ddx + Rb*ddphi - a_l_h/2) / Rw;

a_ll_v = -l_wl*sin(thetall)*ddthetall - l_wl*cos(thetall)*dthetall^2;
a_lr_v = -l_wr*sin(thetalr)*ddthetalr - l_wr*cos(thetalr)*dthetalr^2;

%% § 动力学方程 (06 文档, 消力后的完整非线性式)
% 水平动量 (式 176-178)
eqn1 = (Twl - Iw*ddthetawl + Twr - Iw*ddthetawr) ...
     - (2*mw*Rw*ddthetawl + 2*mw*Rw*ddthetawr ...
     + (ml + mb/2)*(l_wl*cos(thetall)*ddthetall - l_wl*sin(thetall)*dthetall^2 ...
                  + l_wr*cos(thetalr)*ddthetalr - l_wr*sin(thetalr)*dthetalr^2) ...
     + mb/2*(l_bl*cos(thetall)*ddthetall - l_bl*sin(thetall)*dthetall^2 ...
           + l_br*cos(thetalr)*ddthetalr - l_br*sin(thetalr)*dthetalr^2) ...
     + mb*(ddx + l_l/2*cos(thetall)*ddthetall - l_l/2*sin(thetall)*dthetall^2 ...
              + l_r/2*cos(thetalr)*ddthetalr - l_r/2*sin(thetalr)*dthetalr^2));

% yaw (式 191-193)
eqn2 = Iphi*ddphi - ((Twr - Iw*ddthetawr) - (Twl - Iw*ddthetawl))*Rb/Rw;

% 机体俯仰 (式 205-207)
eqn3 = -Tll - Tlr - Ib*ddthetab ...
     + mb*g*db*sin(thetab) - mb*ddx*cos(thetab) ...
     + mb/2*(-l_l*sin(thetall)*ddthetall - l_l*cos(thetall)*dthetall^2 ...
             - l_r*sin(thetalr)*ddthetalr - l_r*cos(thetalr)*dthetalr^2)*db*sin(thetab) ...
     - mb/2*(l_l*cos(thetall)*ddthetall - l_l*sin(thetall)*dthetall^2 ...
             + l_r*cos(thetalr)*ddthetalr - l_r*sin(thetalr)*dthetalr^2)*cos(thetab);

% 左腿转动 (式 227-233)
eqn4 = Tll - Twl ...
     + mb/2*l_l*sin(thetall)*(-l_l/2*sin(thetall)*ddthetall - l_l/2*cos(thetall)*dthetall^2 ...
                              - l_r/2*sin(thetalr)*ddthetalr - l_r/2*cos(thetalr)*dthetalr^2 + g) ...
     + ml/2*l_wl*sin(thetall)*(-l_wl*sin(thetall)*ddthetall - l_wl*cos(thetall)*dthetall^2 + g) ...
     - l_wl*cos(thetall)*((Twl - Iw*ddthetawl)/Rw - mw*ddx) ...
     - mb*l_bl*cos(thetall)*(ddx + l_l*cos(thetall)*ddthetall - l_l*sin(thetall)*dthetall^2) ...
     - Ill*ddthetall;

% 右腿转动 (式 247-253)
eqn5 = Tlr - Twr ...
     + mb/2*l_r*sin(thetalr)*(-l_l/2*sin(thetall)*ddthetall - l_l/2*cos(thetall)*dthetall^2 ...
                              - l_r/2*sin(thetalr)*ddthetalr - l_r/2*cos(thetalr)*dthetalr^2 + g) ...
     + ml/2*l_wr*sin(thetalr)*(-l_wr*sin(thetalr)*ddthetalr - l_wr*cos(thetalr)*dthetalr^2 + g) ...
     - l_wr*cos(thetalr)*((Twr - Iw*ddthetawr)/Rw - mw*ddx) ...
     - mb*l_br*cos(thetalr)*(ddx + l_r*cos(thetalr)*ddthetalr - l_r*sin(thetalr)*dthetalr^2) ...
     - Ilr*ddthetalr;

%% § 平衡点 (无腿质心偏置, 竖直平衡)
thetall_eq = sym(0);
thetalr_eq = sym(0);
thetab_eq  = sym(0);

%% § 代入物理常数
params_subs = {
    g,     p.g;
    Rb,    p.Rb;
    Rw,    p.Rw;
    mb,    p.mb;
    Ib,    p.Ib;
    db,    p.db;
    Iphi,  p.Iphi;
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
fprintf('Computing Jacobians for dual ...\n');
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

fprintf('Generating dual/matrices.m ...\n');
matlabFunction(A_num, B_num, thetall_eq, thetalr_eq, ...
    'File', 'matrices', ...
    'Vars', {ll, lr, thetall0, thetalr0, dll, dlr, Ill, Ilr}, ...
    'Outputs', {'A', 'B', 'thetall_eq', 'thetalr_eq'});
fprintf('Done.\n');
