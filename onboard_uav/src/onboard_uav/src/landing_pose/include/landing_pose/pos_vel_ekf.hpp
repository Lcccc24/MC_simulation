#pragma once

/*
 * This class implements a simple 1-D Extended Kalman Filter to estimate the Relative body frame postion of the lading target and its relative velocity
 * position and velocity of the target is predicted using delta velocity
 * The predictions are corrected periodically using the landing target sensor(or camera)
 */
class PosVelEKF
{
public:
    // Initialize the covariance and state matrix
    // This is called when the landing target is located for the first time or it was lost, then relocated
    // 初始化协方差和状态矩阵，当第一次定位到着陆目标或重新定位时调用
    void init(float pos, float posVar, float vel, float velVar);

    // This functions runs the Prediction Step of the EKF
    // This is called at 400 hz
    // 运行EKF的预测步骤，以预测目标的位置和速度，该函数在400Hz下调用
    void predict(float dt, float dVel, float dVelNoise);

    // fuse the new sensor measurement into the EKF calculations
    // This is called whenever we have a new measurement available
    // 将新的传感器测量融合到EKF计算中，每当有新的测量可用时调用
    void fusePos(float pos, float posVar);

    // Get the EKF state position
    // 获取EKF状态位置
    float getPos() const { return _state[0]; }

    // Get the EKF state velocity
    // 获取EKF状态速度
    float getVel() const { return _state[1]; }

    // get the normalized innovation squared
    // 获取标准化创新平方（Normalized Innovation Squared，NIS）
    float getPosNIS(float pos, float posVar);

private:
    // stored covariance and state matrix

    /*
    _state[0] = position
    _state[1] = velocity
    */
    float _state[2];

    /*
    Covariance Matrix is ALWAYS symmetric, therefore the following matrix is assumed:
    P = Covariance Matrix = |_cov[0]  _cov[1]|
                            |_cov[1]  _cov[2]|
    */
    // 协方差矩阵是对称的，因此使用一个一维数组来存储
    float _cov[3];
};
