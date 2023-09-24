#include "KF.h"

#include <stdio.h>
#include <stdlib.h>

#include "INS_task.h"
#include "cmsis_os.h"
#include "gimbal_task.h"

// Matrix

void Matrix_init(Matrix *p, int n, int m)
{
    p->n = n;
    p->m = m;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            p->val[i][j] = 0;
        }
    }
}

void Matrix_I_init(Matrix *x, int n, int m)
{
    x->n = n;
    x->m = m;
    for (int i = 0; i < n; i++)
    {
        x->val[i][i] = 1.0;
    }
}

void swap_line(Matrix *x, int from, int to)
{
    for (int i = 0; i < x->m; i++)
    {
        fp32 temp = x->val[from][i];
        x->val[from][i] = x->val[to][i];
        x->val[to][i] = temp;
    }
}

void Matrix_copy_to(const Matrix *x, Matrix *y)
{
    y->n = x->n;
    y->m = x->m;
    for (int i = 0; i < x->n; i++)
    {
        for (int j = 0; j < x->m; j++)
        {
            y->val[i][j] = x->val[i][j];
        }
    }
}

void transpose(const Matrix *x, Matrix *ans)
{
    ans->n = x->m;
    ans->m = x->n;
    for (int i = 0; i < ans->n; i++)
    {
        for (int j = 0; j < ans->m; j++)
        {
            ans->val[i][j] = x->val[j][i];
        }
    }
}

void add(const Matrix *x, const Matrix *y, Matrix *ans)
{
    //	if(x->n != y->n || x->m != y->m) waring();
    ans->n = x->n;
    ans->m = x->m;
    for (int i = 0; i < ans->n; i++)
    {
        for (int j = 0; j < ans->m; j++)
        {
            ans->val[i][j] = x->val[i][j] + y->val[i][j];
        }
    }
}

void sub(const Matrix *x, const Matrix *y, Matrix *ans)
{
    //	if(x->n != y->n || x->m != y->m) waring();
    ans->n = x->n;
    ans->m = x->m;
    for (int i = 0; i < ans->n; i++)
    {
        for (int j = 0; j < ans->m; j++)
        {
            ans->val[i][j] = x->val[i][j] - y->val[i][j];
        }
    }
}

void dot(fp32 x, const Matrix *y, Matrix *ans)
{
    ans->n = y->n;
    ans->m = y->m;
    for (int i = 0; i < ans->n; i++)
    {
        for (int j = 0; j < ans->m; j++)
        {
            ans->val[i][j] = y->val[i][j] * x;
        }
    }
}

void mul(const Matrix *x, const Matrix *y, Matrix *ans)
{
    Matrix_init(ans, x->n, y->m);
    //	if(x->m != y->n) waring();
    for (int i = 0; i < ans->n; i++)
    {
        for (int j = 0; j < ans->m; j++)
        {
            for (int k = 0; k < x->m; k++)
            {
                ans->val[i][j] += x->val[i][k] * y->val[k][j];
            }
        }
    }
}

void inv(Matrix x, Matrix *ans)
{
    ans->n = x.n;
    ans->m = x.m;
    for (int i = 0; i < x.n; i++)
    {
        ans->val[i][i] = 1.0;
    }
    for (int i = 0; i < x.n; i++)
    {
        int maxArg = i;
        for (int j = i + 1; j < x.n; j++)
            if (x.val[j][i] > x.val[maxArg][i])
                maxArg = j;
        if (maxArg != i)
        {
            swap_line(&x, maxArg, i);
            swap_line(ans, maxArg, i);
        }
        if (x.val[i][i] == 0)
            return;
        for (int j = 0; j < x.n; j++)
        {
            if (i == j)
                continue;
            fp32 c = x.val[j][i] / x.val[i][i];
            for (int k = i; k < x.m; k++)
                x.val[j][k] -= x.val[i][k] * c;
            for (int k = 0; k < x.m; k++)
                ans->val[j][k] -= ans->val[i][k] * c;
        }
    }
    for (int i = 0; i < x.n; i++)
    {
        for (int j = 0; j < x.m; j++)
        {
            ans->val[i][j] /= x.val[i][i];
        }
    }
}

// Kalman_Filter

void kalman_init(Kalman_Filter *Kalman_config, int dim_x, int dim_p, Matrix Tr, Matrix H, Matrix B, fp32 Q, fp32 R)
{
    Kalman_config->H = H;
    Kalman_config->Tr = Tr;
    Kalman_config->B = B;

    Kalman_config->dim_x = dim_x;
    Kalman_config->dim_p = dim_p;

    Matrix_init(&Kalman_config->x, dim_x, 1);
    Matrix_I_init(&Kalman_config->K, dim_x, dim_x);
    Matrix_I_init(&Kalman_config->I, dim_x, dim_x);
    Matrix_I_init(&Kalman_config->Q, dim_x, dim_x);
    Matrix_I_init(&Kalman_config->R, dim_p, dim_p);
    Matrix_I_init(&Kalman_config->P, dim_x, dim_x);

    transpose(&Kalman_config->Tr, &Kalman_config->Trt);
    transpose(&Kalman_config->H, &Kalman_config->Ht);

    dot(Q, &Kalman_config->Q, &Kalman_config->Q);
    dot(R, &Kalman_config->R, &Kalman_config->R);
}

void kalman_clear(Kalman_Filter *Kalman_config)
{
    Matrix_init(&Kalman_config->x, Kalman_config->dim_x, 1);
    Matrix_I_init(&Kalman_config->P, Kalman_config->dim_x, Kalman_config->dim_x);
}

void Kalman_Calc(Kalman_Filter *kf, Matrix *z, Matrix *u)
{
    // E1
    mul(&kf->Tr, &kf->x, &kf->t1);
    mul(&kf->B, u, &kf->t2);
    add(&kf->t1, &kf->t2, &kf->x);

    // E2
    mul(&kf->Tr, &kf->P, &kf->t1);
    mul(&kf->t1, &kf->Trt, &kf->t2);
    add(&kf->t2, &kf->Q, &kf->P);

    // E3
    mul(&kf->P, &kf->Ht, &kf->t3);
    mul(&kf->H, &kf->t3, &kf->t1);
    add(&kf->t1, &kf->R, &kf->t2);
    inv(kf->t2, &kf->t1);
    mul(&kf->t3, &kf->t1, &kf->K);

    // E4
    mul(&kf->H, &kf->x, &kf->t1);
    sub(z, &kf->t1, &kf->t2);
    mul(&kf->K, &kf->t2, &kf->t1);
    add(&kf->x, &kf->t1, &kf->x);
    //	add(x, mul(K, sub(z, mul(H, x))))

    // E5
    mul(&kf->K, &kf->H, &kf->t1);
    sub(&kf->I, &kf->t1, &kf->t2);
    mul(&kf->t2, &kf->P, &kf->t1);
    Matrix_copy_to(&kf->t1, &kf->P);
}

char vis = 0, need_init = 0;
fp32 data_x = 0.0, data_y = 0.0, control_data = 0.0;

Kalman_Filter kf;

void KF_set_calc(fp32 x, fp32 y)
{
    vis = 1;
    data_x = x;
    data_y = y;
}

void KF_clear(void)
{
    kalman_clear(&kf);
}

void KF_set_contorl(fp32 x)
{
    control_data = x;
}

void KF_set_init(void)
{
    need_init = 1;
}

const Matrix *get_out(void)
{
    return &kf.x;
}

void KF_task(void const *pvParameters)
{
    // 状态维度，迭代次数
    int dim = 3;
    // 离散时间
    fp32 dt = 0.025;
    // 状态转移矩阵
    Matrix Tr = { dim, dim, { { 1.0, dt, dt * dt * 0.5 }, { 0.0, 1.0, dt }, { 0.0, 0.0, 1.0 } } };
    // 传感器矩阵
    Matrix H = { 1, dim, { { 1, 0, 0 } } };
    // 控制矩阵
    Matrix B = { dim, 1, { { 1.0f }, { 0.0f }, { 0.0f } } };
    kalman_init(&kf, dim, 1, Tr, H, B, 1e-1, 5e-2);

    static Matrix z, u;
    Matrix_init(&z, 1, 1);
    Matrix_init(&u, 1, 1);

    const fp32 pre_time = 0.5;

    while (1)
    {
        if (vis)
        {
            if (need_init == 1)
            {
                kalman_clear(&kf);
                gimbal_cv_pid_clear();
                kf.x.val[0][0] = data_x;
                need_init = 0;
            }
            z.val[0][0] = data_x;
            u.val[0][0] = -control_data * dt * -15.0;
            control_data = 0;
            Kalman_Calc(&kf, &z, &u);
            gimbal_cv_pid_calc(
                kf.x.val[0][0] + pre_time * kf.x.val[1][0] + 0.5 * pre_time * pre_time * kf.x.val[2][0], data_y);
            // gimbal_cv_pid_calc(data_x, data_y);
            vis = 0;
        }
        vTaskDelay(1);
    }
}
