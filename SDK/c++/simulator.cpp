#include "simulator.h"
#include "log.h"

using namespace Geometry;

void Simulator::SimuAFrame(Robot& robot, double forward, double rotate, double per) {
    double linearV = Geometry::Length(robot.linear_velocity_);
    double angularV = robot.angular_velocity_;
    double orient = robot.orient_;
    Point pos = robot.pos_;
    const double rate = 0.84; // [0.79, 0.90]
    double changev = per * robot.max_force_ / robot.GetMass() * rate;
    double changer = per * robot.max_rot_force_ / robot.GetRotInerta() * rate;
    for (int i = 0; i < 1; i++) {
        // update
        if (fabs(linearV - forward) > changev) {
            if (forward > linearV)
                linearV += changev;
            else 
                linearV -= changev;
        } else linearV = forward;

        if (fabs(angularV - rotate) > changer) {
            if (rotate > angularV)
                angularV += changer;
            else 
                angularV -= changer;
        } else angularV = rotate;
        orient = Geometry::AngleReg(orient + angularV * per);

        linearV = std::max(1e-10, linearV);
        angularV = std::max(1e-10, fabs(angularV)) * (angularV > 0 ? 1 : -1);

        double cir = linearV / fabs(angularV);
        Vector mov = Vector{-sin(orient), cos(orient)} * cir;
        Point center = pos + mov;
        pos = center - Rotate(mov, per * linearV / cir);

        // Log::print("angularV", angularV);
        // Log::print("linearV", linearV);
        // Log::print("pos", pos);
    }
    robot.pos_ = pos;
    robot.orient_ = orient;
    robot.angular_velocity_ = angularV;
    robot.linear_velocity_ = Vector{cos(orient), sin(orient)} * linearV;
}
std::vector<Geometry::Point> Simulator::SimuFrames(Robot robot, std::function<std::pair<double,double>()> action, int frames, int sampling) {
    std::vector<Geometry::Point> ans;
    for (int i = 0; i < frames; i++) {
        auto [forward, rotate] = action();
        forward = std::min(robot.max_forward_velocity_, std::max(robot.max_backward_velocity_, forward));
        rotate = std::min(robot.max_rotate_velocity_, std::max(-robot.max_rotate_velocity_, rotate));
        SimuAFrame(robot, forward, rotate, 1.0 / 50);
        if (i % sampling == 0) ans.push_back(robot.pos_);
    }
    return ans;
}