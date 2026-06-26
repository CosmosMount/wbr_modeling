

#pragma once

#include "config_chassis.hpp"

namespace chassis
{
/**
 * Offset parallel VMC (偏置并联): direct (l, phi) kinematics.
 * See 04_vmc.md §偏置并联VMC.
 */
class VMCParallel
{
protected:
    float J_mat[4] = {0};
    float JT_mat[4] = {0};
    float JT_inv_mat[4] = {0};

    /* ∂l_s/∂φ₁, ∂l_s/∂φ₄ */
    float Js_mat[2] = {0.0f, 0.0f};

    float phi1 = 0.0f;
    float phi4 = 0.0f;

    float len = 0.0f;
    float phi = 0.0f;
    float alpha = 0.0f;

    float Fs = 0.0f;

    static float ClampAcosArg(float x)
    {
        if (x > 1.0f)
            return 1.0f;
        if (x < -1.0f)
            return -1.0f;
        return x;
    }

    void CalcJs()
    {
        float alpha_s = acosf(ClampAcosArg(
            (Dspring3 * Dspring3 + L1 * L1 - Dspring1 * Dspring1) / (2.0f * Dspring3 * L1)));
        float theta = acosf(ClampAcosArg((L1 * L1 + L2 * L2 - len * len) / (2.0f * L1 * L2)));
        float beta_s = theta - alpha_s;

        float ls = sqrtf(Dspring2 * Dspring2 + Dspring3 * Dspring3 -
                         2.0f * Dspring2 * Dspring3 * arm_cos_f32(beta_s));
        float sin_theta = arm_sin_f32(theta);
        if (fabsf(sin_theta) < 1e-5f || ls < 1e-5f)
        {
            Js_mat[0] = Js_mat[1] = 0.0f;
            return;
        }

        float dl_s_dl = Dspring2 * Dspring3 * len * arm_sin_f32(beta_s) /
                        (L1 * L2 * ls * sin_theta);
        Js_mat[0] = dl_s_dl * J_mat[0];
        Js_mat[1] = dl_s_dl * J_mat[1];
    }

    void CalcSpringForce()
    {
        float tau_s1 = Fspring * Js_mat[0];
        float tau_s4 = Fspring * Js_mat[1];

        Fs = JT_inv_mat[0] * tau_s1 + JT_inv_mat[1] * tau_s4;
    }

public:
    void Resolve(float _phi1, float _phi4)
    {
        phi1 = _phi1;
        phi4 = _phi4;

        alpha = (phi1 - phi4) * 0.5f;
        phi = (phi1 + phi4) * 0.5f;

        float sin_alpha = arm_sin_f32(alpha);
        float cos_alpha = arm_cos_f32(alpha);
        float sqrt_term = sqrtf(L2 * L2 - L1 * L1 * sin_alpha * sin_alpha);
        len = L1 * cos_alpha + sqrt_term;

        float denom = L1 * cos_alpha - len;
        if (fabsf(denom) < 1e-5f)
        {
            J_mat[0] = J_mat[1] = J_mat[2] = J_mat[3] = 0.0f;
            JT_mat[0] = JT_mat[1] = JT_mat[2] = JT_mat[3] = 0.0f;
            JT_inv_mat[0] = JT_inv_mat[1] = JT_inv_mat[2] = JT_inv_mat[3] = 0.0f;
            Js_mat[0] = Js_mat[1] = 0.0f;
            Fs = 0.0f;
            return;
        }

        float j_scale = len * L1 * sin_alpha / (2.0f * denom);

        J_mat[0] = j_scale;
        J_mat[1] = -j_scale;
        J_mat[2] = 0.5f;
        J_mat[3] = 0.5f;

        JT_mat[0] = J_mat[0];
        JT_mat[1] = J_mat[2];
        JT_mat[2] = J_mat[1];
        JT_mat[3] = J_mat[3];

        if (fabsf(j_scale) < 1e-5f)
        {
            JT_inv_mat[0] = JT_inv_mat[1] = JT_inv_mat[2] = JT_inv_mat[3] = 0.0f;
        }
        else
        {
            JT_inv_mat[0] = 0.5f / j_scale;
            JT_inv_mat[1] = -0.5f / j_scale;
            JT_inv_mat[2] = 1.0f;
            JT_inv_mat[3] = 1.0f;
        }

        CalcJs();
        CalcSpringForce();
    }

    void VMCCal(float *F, float *T)
    {
        T[0] = JT_mat[0] * F[0] + JT_mat[1] * F[1];
        T[1] = JT_mat[2] * F[0] + JT_mat[3] * F[1];
    }

    void VMCRevCal(float *F, float *T)
    {
        F[0] = JT_inv_mat[0] * T[0] + JT_inv_mat[1] * T[1];
        F[1] = JT_inv_mat[2] * T[0] + JT_inv_mat[3] * T[1];
    }

    void VMCVelCal(float *phi_dot, float *v_dot)
    {
        v_dot[0] = J_mat[0] * phi_dot[0] + J_mat[1] * phi_dot[1];
        v_dot[1] = J_mat[2] * phi_dot[0] + J_mat[3] * phi_dot[1];
    }

    inline float GetLen() { return len; }
    inline float GetPhi() { return phi; }
    inline float GetPhi4() { return phi4; }
    inline float GetPhi1() { return phi1; }
    inline float GetAlpha() { return alpha; }
    inline float GetFs() { return Fs; }
};
} // namespace chassis
