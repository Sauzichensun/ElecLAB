#include "math.h"
#include "WIE.h"


// ============ 四元数结构体 ============
typedef struct {
    float q0, q1, q2, q3;  // 四元数 [w, x, y, z]
} Quaternion_t;

// ============ 全局变量 ============
static Quaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f};  // 初始化为单位四元数
static float sampleFreq = 1000.0f;  // 采样频率 (Hz)
static float beta = 0.1f;          // Madgwick算法增益参数（建议0.033~0.5）

// ============ 快速平方根倒数 ============
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// ============ Madgwick六轴姿态更新 ============
/**
 * @brief Madgwick AHRS算法 - 六轴版本
 * @param gx, gy, gz: 陀螺仪角速度 (弧度/秒)
 * @param ax, ay, az: 加速度计原始值 (任意单位，算法内会归一化)
 */
void Madgwick_Update6DOF(float gx, float gy, float gz, 
                         float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    // 1. 如果加速度计数据无效，只用陀螺仪积分
    if((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
        // 纯陀螺仪积分（会漂移）
        qDot1 = 0.5f * (-q.q1 * gx - q.q2 * gy - q.q3 * gz);
        qDot2 = 0.5f * ( q.q0 * gx + q.q2 * gz - q.q3 * gy);
        qDot3 = 0.5f * ( q.q0 * gy - q.q1 * gz + q.q3 * gx);
        qDot4 = 0.5f * ( q.q0 * gz + q.q1 * gy - q.q2 * gx);

        q.q0 += qDot1 * (1.0f / sampleFreq);
        q.q1 += qDot2 * (1.0f / sampleFreq);
        q.q2 += qDot3 * (1.0f / sampleFreq);
        q.q3 += qDot4 * (1.0f / sampleFreq);

        // 归一化四元数
        recipNorm = invSqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
        q.q0 *= recipNorm;
        q.q1 *= recipNorm;
        q.q2 *= recipNorm;
        q.q3 *= recipNorm;
        return;
    }

    // 2. 加速度计归一化
    recipNorm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 3. 预计算常用项
    _2q0 = 2.0f * q.q0;
    _2q1 = 2.0f * q.q1;
    _2q2 = 2.0f * q.q2;
    _2q3 = 2.0f * q.q3;
    _4q0 = 4.0f * q.q0;
    _4q1 = 4.0f * q.q1;
    _4q2 = 4.0f * q.q2;
    _8q1 = 8.0f * q.q1;
    _8q2 = 8.0f * q.q2;
    q0q0 = q.q0 * q.q0;
    q1q1 = q.q1 * q.q1;
    q2q2 = q.q2 * q.q2;
    q3q3 = q.q3 * q.q3;

    // 4. 梯度下降算法（优化目标函数）
    // 目标：使四元数表示的重力方向与加速度计测量对齐
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q.q3 - _2q1 * ax + 4.0f * q2q2 * q.q3 - _2q2 * ay;

    // 归一化梯度
    recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // 5. 计算四元数变化率（陀螺仪积分 - beta*梯度修正）
    qDot1 = 0.5f * (-q.q1 * gx - q.q2 * gy - q.q3 * gz) - beta * s0;
    qDot2 = 0.5f * ( q.q0 * gx + q.q2 * gz - q.q3 * gy) - beta * s1;
    qDot3 = 0.5f * ( q.q0 * gy - q.q1 * gz + q.q3 * gx) - beta * s2;
    qDot4 = 0.5f * ( q.q0 * gz + q.q1 * gy - q.q2 * gx) - beta * s3;

    // 6. 积分四元数
    q.q0 += qDot1 * (1.0f / sampleFreq);
    q.q1 += qDot2 * (1.0f / sampleFreq);
    q.q2 += qDot3 * (1.0f / sampleFreq);
    q.q3 += qDot4 * (1.0f / sampleFreq);

    // 7. 四元数归一化
    recipNorm = invSqrt(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
    q.q0 *= recipNorm;
    q.q1 *= recipNorm;
    q.q2 *= recipNorm;
    q.q3 *= recipNorm;
}

// ============ 四元数转欧拉角 ============
/**
 * @brief 将四元数转换为欧拉角
 * @param roll, pitch, yaw: 输出角度（度）
 */
void Madgwick_GetEulerAngles(float *roll, float *pitch, float *yaw)
{
    // Roll (绕X轴)
    *roll = atan2f(2.0f * (q.q0*q.q1 + q.q2*q.q3),
                   1.0f - 2.0f * (q.q1*q.q1 + q.q2*q.q2)) * 57.2957795f;

    // Pitch (绕Y轴)
    float sinp = 2.0f * (q.q0*q.q2 - q.q3*q.q1);
    if (fabsf(sinp) >= 1.0f)
        *pitch = copysignf(90.0f, sinp);  // 万向锁保护
    else
        *pitch = asinf(sinp) * 57.2957795f;

    // Yaw (绕Z轴)
    *yaw = atan2f(2.0f * (q.q0*q.q3 + q.q1*q.q2),
                  1.0f - 2.0f * (q.q2*q.q2 + q.q3*q.q3)) * 57.2957795f;
}

// ============ 获取四元数 ============
void Madgwick_GetQuaternion(float *q0, float *q1, float *q2, float *q3)
{
    *q0 = q.q0;
    *q1 = q.q1;
    *q2 = q.q2;
    *q3 = q.q3;
}

// ============ 设置采样频率 ============
void Madgwick_SetSampleFrequency(float freq)
{
    sampleFreq = freq;
}

// ============ 设置Beta增益 ============
/**
 * @param newBeta: 建议范围 0.033~0.5
 *   - 值越大：收敛越快，但噪声更多
 *   - 值越小：滤波平滑，但响应慢
 */
void Madgwick_SetBeta(float newBeta)
{
    beta = newBeta;
}

// ============ 重置姿态 ============
void Madgwick_Reset(void)
{
    q.q0 = 1.0f;
    q.q1 = 0.0f;
    q.q2 = 0.0f;
    q.q3 = 0.0f;
}
