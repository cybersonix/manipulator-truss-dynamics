#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "chrono/physics/ChLinkMotorRotation.h"
#include "chrono/functions/ChFunction.h"
#include <vector>
#include <memory>
#include <algorithm>

using namespace chrono;

class JointController
{
public:
    virtual ~JointController() = default;
    // 计算力矩，直接从电机获取当前状态
    virtual double ComputeTorque(std::shared_ptr<ChLinkMotorRotation> motor) = 0;
    virtual void Initialize(const std::vector<double> &params) = 0;
};

class PDController : public JointController
{
public:
    double target_angle = 0;
    double Kp = 0, Kd = 0;
    double max_torque = 100.0;

    void Initialize(const std::vector<double> &params) override
    {
        target_angle = params[0];
        Kp = params[1];
        Kd = params[2];
        if (params.size() > 3)
            max_torque = params[3];
    }

    double ComputeTorque(std::shared_ptr<ChLinkMotorRotation> motor) override
    {
        if (!motor)
            return 0.0;
        double error = target_angle - motor->GetMotorAngle();
        double vel = motor->GetMotorAngleDt();
        double torque = Kp * error - Kd * vel;
        return std::clamp(torque, -max_torque, max_torque);
    }
};

// 将JointController适配到ChFunction的类, 用于在URDF解析时传递给SetMotorFunction
class ControllerChFunction : public ChFunction
{
public:
    ControllerChFunction(std::shared_ptr<JointController> controller,
                         std::shared_ptr<ChLinkMotorRotation> motor)
        : controller_(controller), motor_(motor) {}

    virtual double GetVal(double x) const override
    {
        if (!controller_ || !motor_)
            return 0.0;
        return controller_->ComputeTorque(motor_);
    }

    virtual ChFunction *Clone() const override
    {
        return new ControllerChFunction(*this);
    }

private:
    std::shared_ptr<JointController> controller_;
    std::shared_ptr<ChLinkMotorRotation> motor_;
};

#endif