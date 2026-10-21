#pragma once
#include <cmath>
#include <vector>
#include <stdexcept>
#include <mujoco/mujoco.h>

inline int ElasticBandBody(const mjModel* model) {
  for (const char* name : {"torso_link", "base_link"}) {
    const int body = mj_name2id(model, mjOBJ_BODY, name);
    if (body > 0) return body;
  }
  for (int i = 0; i < model->njnt; ++i)
    if (model->jnt_type[i] == mjJNT_FREE) return model->jnt_bodyid[i];
  throw std::runtime_error("Elastic band requires a robot body");
}

class ElasticBand
{
public:
  ElasticBand(){};
  void Advance(std::vector<double> x, std::vector<double> dx)
  {
    std::vector<double> delta_x = {0.0, 0.0, 0.0};
    delta_x[0] = point_[0] - x[0];
    delta_x[1] = point_[1] - x[1];
    delta_x[2] = point_[2] - x[2];
    double distance = sqrt(delta_x[0] * delta_x[0] + delta_x[1] * delta_x[1] + delta_x[2] * delta_x[2]);
    if(distance<1e-9) {f_={0,0,0};return;}

    std::vector<double> direction = {0.0, 0.0, 0.0};
    direction[0] = delta_x[0] / distance;
    direction[1] = delta_x[1] / distance;
    direction[2] = delta_x[2] / distance;

    double v = dx[0] * direction[0] + dx[1] * direction[1] + dx[2] * direction[2];

    f_[0] = (stiffness_ * (distance - length_) - damping_ * v) * direction[0];
    f_[1] = (stiffness_ * (distance - length_) - damping_ * v) * direction[1];
    f_[2] = (stiffness_ * (distance - length_) - damping_ * v) * direction[2];
  }

  // xfrc_applied acts at the body's center of mass, in world coordinates.
  // Call after mj_forward so position and spatial velocity describe the current state.
  void Apply(const mjModel* model, mjData* data) {
    const int body = ElasticBandBody(model);
    if (enable_) {
      mjtNum velocity[6];
      mj_objectVelocity(model, data, mjOBJ_BODY, body, velocity, 0);
      const auto* position = data->xipos + 3*body;
      Advance({position[0], position[1], position[2]},
              {velocity[3], velocity[4], velocity[5]});
    } else {
      f_ = {0, 0, 0};
    }
    for (int i = 0; i < 3; ++i) data->xfrc_applied[6*body+i] = f_[i];
  }


  double stiffness_ = 200;
  double damping_ = 100;
  std::vector<double> point_ = {0, 0, 3};
  double length_ = 0.0;
  bool enable_ = true;
  std::vector<double> f_ = {0, 0, 0};
};
