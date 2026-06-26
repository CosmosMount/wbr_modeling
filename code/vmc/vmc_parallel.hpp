#pragma once

#include "config_chassis.hpp"

namespace chassis
{
/**
 * Five-link VMC (五连杆): task space (l, phi) via Cartesian (x_C, y_C).
 * See 04_vmc.md §五连杆VMC.
 */
class VMCLink
{
protected:
    float J_mat[4] = {0};
    float JT_mat[4] = {0};
    float JT_inv_mat[4] = {0};

    float Js_mat[4] = {0};
    float JsT_mat[4] = {0};

    float phi1 = 0.0f;
    float phi4 = 0.0f;

    float len = 0.0f;
    float phi = PI / 2;
    float U2 = 0.0f;
    float U3 = 0.0f;

    float CoorC[2] = {0.0f, 0.0f};

    float Fs = 0.0f;

    void CalcJs(float sin32)
    {
        float sin1 = arm_sin_f32(phi1);
        float cos1 = arm_cos_f32(phi1);
        float sin2 = arm_sin_f32(U2);
        float cos2 = arm_cos_f32(U2);
        float ang_s = phi1 + Ang_spring;
        float inv_L2sin32 = 1.0f / (L2 * sin32);

        float term1 = L1 * sin1 + Dspring1 * arm_cos_f32(ang_s);
        float term2 = L1 * cos1 - Dspring1 * arm_sin_f32(ang_s);
        float sin31 = arm_sin_f32(U3 - phi1);
        float sin43 = arm_sin_f32(phi4 - U3);

        Js_mat[0] = -term1 - Dspring2 * L1 * sin31 * sin2 * inv_L2sin32;
        Js_mat[1] = -Dspring2 * L1 * sin43 * sin2 * inv_L2sin32;
        Js_mat[2] = term2 + Dspring2 * L1 * sin31 * cos2 * inv_L2sin32;
        Js_mat[3] = Dspring2 * L1 * sin43 * cos2 * inv_L2sin32;

        JsT_mat[0] = Js_mat[0];
        JsT_mat[1] = Js_mat[2];
        JsT_mat[2] = Js_mat[1];
        JsT_mat[3] = Js_mat[3];
    }

    void CalcSpringForce()
    {
        float cos1 = arm_cos_f32(phi1);
        float sin1 = arm_sin_f32(phi1);
        float cos2 = arm_cos_f32(U2);
        float sin2 = arm_sin_f32(U2);
        float ang_s = phi1 + Ang_spring;

        float dx = L1 * cos1 + Dspring2 * cos2 - Dspring1 * arm_sin_f32(ang_s);
        float dy = L1 * sin1 + Dspring2 * sin2 + Dspring1 * arm_cos_f32(ang_s);
        float ls = sqrtf(dx * dx + dy * dy);
        if (ls < 1e-5f)
        {
            Fs = 0.0f;
            return;
        }

        float Fsx = Fspring * dx / ls;
        float Fsy = Fspring * dy / ls;

        float tau_s1 = JsT_mat[0] * Fsx + JsT_mat[1] * Fsy;
        float tau_s4 = JsT_mat[2] * Fsx + JsT_mat[3] * Fsy;

        Fs = JT_inv_mat[0] * tau_s1 + JT_inv_mat[1] * tau_s4;
    }

public:
    void Resolve(float _phi1, float _phi4)
    {
        phi1 = _phi1;
        phi4 = _phi4;

        float SIN1 = arm_sin_f32(phi1);
        float COS1 = arm_cos_f32(phi1);
        float SIN4 = arm_sin_f32(phi4);
        float COS4 = arm_cos_f32(phi4);

        float xdb = L1 * (COS4 - COS1);
        float ydb = L1 * (SIN4 - SIN1);

        float A0 = 2.0f * L2 * xdb;
        float B0 = 2.0f * L2 * ydb;
        float C0 = xdb * xdb + ydb * ydb;

        float u2t = 0.0f;
        arm_atan2_f32((B0 + sqrtf(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0), &u2t);
        U2 = 2.0f * u2t;

        CoorC[0] = L1 * COS1 + L2 * arm_cos_f32(U2) - d;
        CoorC[1] = L1 * SIN1 + L2 * arm_sin_f32(U2);

        float CoorD[2] = {L1 * COS4, L1 * SIN4};

        float u3t = 0.0f;
        arm_atan2_f32((CoorD[1] - CoorC[1]), (CoorD[0] - CoorC[0]), &u3t);
        U3 = PI + u3t;

        arm_atan2_f32(CoorC[1], CoorC[0], &phi);
        len = sqrtf(CoorC[0] * CoorC[0] + CoorC[1] * CoorC[1]);

        float sin32 = arm_sin_f32(U3 - U2);
        if (fabsf(sin32) < 0.05f)
        {
            J_mat[0] = J_mat[1] = J_mat[2] = J_mat[3] = 0.0f;
            JT_mat[0] = JT_mat[1] = JT_mat[2] = JT_mat[3] = 0.0f;
            JT_inv_mat[0] = JT_inv_mat[1] = JT_inv_mat[2] = JT_inv_mat[3] = 0.0f;
            Js_mat[0] = Js_mat[1] = Js_mat[2] = Js_mat[3] = 0.0f;
            JsT_mat[0] = JsT_mat[1] = JsT_mat[2] = JsT_mat[3] = 0.0f;
            Fs = 0.0f;
            return;
        }

        CalcJs(arm_sin_f32(U2 - U3));

        float sin12 = arm_sin_f32(phi1 - U2);
        float sin34 = arm_sin_f32(U3 - phi4);
        float cos03 = arm_cos_f32(phi - U3);
        float cos02 = arm_cos_f32(phi - U2);
        float sin03 = arm_sin_f32(phi - U3);
        float sin02 = arm_sin_f32(phi - U2);

        J_mat[0] = L1 * sin03 * sin12 / sin32;
        J_mat[1] = L1 * sin02 * sin34 / sin32;
        J_mat[2] = L1 * cos03 * sin12 / (sin32 * len);
        J_mat[3] = L1 * cos02 * sin34 / (sin32 * len);

        JT_mat[0] = J_mat[0];
        JT_mat[1] = J_mat[2];
        JT_mat[2] = J_mat[1];
        JT_mat[3] = J_mat[3];

        JT_inv_mat[0] = -cos02 / (sin12 * L1);
        JT_inv_mat[1] = cos03 / (sin34 * L1);
        JT_inv_mat[2] = sin02 * len / (sin12 * L1);
        JT_inv_mat[3] = -sin03 * len / (sin34 * L1);

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
    inline float GetFs() { return Fs; }
};
} // namespace chassis