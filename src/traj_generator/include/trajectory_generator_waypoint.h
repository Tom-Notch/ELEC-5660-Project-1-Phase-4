#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <eigen3/Eigen/Dense>
#include <vector>
class TrajectoryGeneratorWaypoint
{
private:
  double _qp_cost;
  Eigen::MatrixXd _Q;
  Eigen::VectorXd _Px, _Py, _Pz;

public:
  TrajectoryGeneratorWaypoint();

  ~TrajectoryGeneratorWaypoint();

  Eigen::MatrixXd PolyQPGeneration(
      const Eigen::MatrixXd& Path,
      const Eigen::MatrixXd& Vel,
      const Eigen::MatrixXd& Acc,
      const Eigen::VectorXd& Time);

  double getObjective();
};

#endif
