// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <cmath>
#include <memory>
#include <unordered_map>

#include "simulator.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;
using std::size_t;

class BearingSlam {
 public:
  BearingSlam();
  ~BearingSlam();

  void AddOdometry(const Simulator::GridEdge& odom);
  void AddObservation(const Simulator::LandmarkEdge& obs);
  void Optimize(const int iterations);

  g2o::SE2 GetLatestEstimate() const { return poses_.at(last_pose_id_); }

 private:
  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  std::unique_ptr<SlamLinearSolver> linear_solver_;
  std::unique_ptr<SlamBlockSolver> block_solver_;
  std::unique_ptr<g2o::OptimizationAlgorithmGaussNewton> lm_solver_;
  g2o::SparseOptimizer optimizer_;

  g2o::SE2 sensor_offset_tf_;
  std::unique_ptr<g2o::ParameterSE2Offset> sensor_offset_;

  std::unordered_map<size_t, g2o::SE2> poses_;
  std::unordered_map<size_t, g2o::Vector2D> landmarks_;
  size_t last_pose_id_;
};

BearingSlam::BearingSlam() {
  // Allocate solvers
  linear_solver_.reset(new SlamLinearSolver());
  linear_solver_->setBlockOrdering(false);
  block_solver_.reset(new SlamBlockSolver(linear_solver_.get()));
  lm_solver_.reset(new g2o::OptimizationAlgorithmGaussNewton(block_solver_.get()));
  optimizer_.setAlgorithm(lm_solver_.get());
  optimizer_.setVerbose(true);

  // Configure sensor offset
  sensor_offset_.reset(new g2o::ParameterSE2Offset());
  sensor_offset_tf_ = g2o::SE2(0.2, 0.1, -0.1);
  sensor_offset_->setOffset(sensor_offset_tf_);
  sensor_offset_->setId(0);

  // Add first pose
  last_pose_id_ = 0;
  poses_[last_pose_id_] = g2o::SE2();
  g2o::VertexSE2* first_pose = new g2o::VertexSE2;
  first_pose->setId(last_pose_id_);
  first_pose->setEstimate(poses_[last_pose_id_]);
  first_pose->setFixed(true);  // fix first pose
  optimizer_.addVertex(first_pose);
}

BearingSlam::~BearingSlam() {
  optimizer_.clear();

  // Release objects in reverse order
  lm_solver_.release();
  block_solver_.release();
  linear_solver_.release();

  Factory::destroy();
  OptimizationAlgorithmFactory::destroy();
}

void BearingSlam::AddOdometry(const Simulator::GridEdge& odom) {
  if (poses_.count(odom.from) > 0) {
    // Lookup previous pose
    const g2o::SE2& prev_pose = poses_[odom.from];

    // Create new pose if neccessary
    if (poses_.count(odom.to) == 0) {
      const g2o::SE2 pred_pose = prev_pose * odom.simulatorTransf;

      last_pose_id_ = odom.to;
      poses_[odom.to] = pred_pose;

      g2o::VertexSE2* v = new g2o::VertexSE2;
      v->setId(odom.to);
      v->setEstimate(pred_pose);
      optimizer_.addVertex(v);
    }

    // Add the odometry edge
    g2o::EdgeSE2* edge = new g2o::EdgeSE2;
    edge->vertices()[0] = optimizer_.vertex(odom.from);
    edge->vertices()[1] = optimizer_.vertex(odom.to);
    edge->setMeasurement(odom.simulatorTransf);
    edge->setInformation(odom.information);
    optimizer_.addEdge(edge);
  } else {
    std::cerr << "FUCK. Base pose in AddOdometry doesn't exist yet!\n";
  }
}

void BearingSlam::AddObservation(const Simulator::LandmarkEdge& obs) {
  if (poses_.count(obs.from) > 0) {
    // Lookup base pose
    const g2o::SE2& prev_pose = poses_[obs.from];

    // Initialize landmark if necessary
    if (landmarks_.count(obs.to) == 0) {
      const g2o::Vector2D pred_lm = prev_pose * obs.simulatorMeas;
      landmarks_[obs.to] = pred_lm;
      g2o::VertexPointXY* v = new g2o::VertexPointXY;
      v->setId(obs.to);
      v->setEstimate(pred_lm);
      optimizer_.addVertex(v);
    }

    g2o::EdgeSE2PointXY* edge = new g2o::EdgeSE2PointXY;
    edge->vertices()[0] = optimizer_.vertex(obs.from);
    edge->vertices()[1] = optimizer_.vertex(obs.to);
    edge->setMeasurement(obs.simulatorMeas);
    edge->setInformation(obs.information);
    edge->setParameterId(0, sensor_offset_->id());
    optimizer_.addEdge(edge);
  } else {
    std::cerr << "FUCK. Base pose in AddObservation doesn't exist yet!\n";
  }
}

void BearingSlam::Optimize(const int iterations) {
  // Optimize
  optimizer_.initializeOptimization();
  optimizer_.optimize(iterations);

  // Update state
  const g2o::SparseOptimizer::VertexContainer& vertices = optimizer_.activeVertices();
  for (auto itr = vertices.cbegin(); itr != vertices.cend(); ++itr) {
    const int id = (**itr).id();
    if (poses_.count(id) > 0) {
      double data[3];
      dynamic_cast<g2o::VertexSE2*>(*itr)->getEstimateData(data);
      poses_[id] = g2o::SE2(data[0], data[1], data[2]);
    } else if (landmarks_.count(id) > 0) {
      double data[2];
      dynamic_cast<g2o::VertexPointXY*>(*itr)->getEstimateData(data);
      landmarks_[id] << data[0], data[1];
    } else {
      std::cerr << "FUCK. Unrecognized vertex id.\n";
    }
  }
}

int main()
{
  // TODO simulate different sensor offset
  // simulate a robot observing landmarks while travelling on a grid
  SE2 sensorOffsetTransf(0.2, 0.1, -0.1);
  int numNodes = 10;
  Simulator simulator;
  simulator.simulate(numNodes, sensorOffsetTransf);

  BearingSlam bs;

  // Walk through one pose at a time
  auto itr_p = simulator.poses().cbegin() + 1;  // start off by one
  auto itr_odom = simulator.odometry().cbegin();
  auto itr_obs = simulator.landmarkObservations().cbegin();
  for (; itr_p != simulator.poses().cend(); ++itr_p, ++itr_odom) {
    bs.AddOdometry(*itr_odom);
    while (itr_obs != simulator.landmarkObservations().cend() && itr_obs->from <= itr_p->id) {
      bs.AddObservation(*itr_obs);
      ++itr_obs;
    }

    bs.Optimize(5);
    g2o::SE2 est_pose, true_pose;
    est_pose = bs.GetLatestEstimate();
    true_pose = itr_p->truePose;
    std::cout << "Estimate SE2: " << est_pose.toVector().transpose() << std::endl;
    std::cout << "True SE2: " << true_pose.toVector().transpose() << std::endl;
    std::cout << "\n\n\n";
  }

  return 0;

}
