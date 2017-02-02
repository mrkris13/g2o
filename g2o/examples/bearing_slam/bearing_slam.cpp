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
    g2o::SE2 prev_pose = poses_[odom.from];

    // Create new pose if neccessary
    if (poses_.count(odom.to) == 0) {
      g2o::SE2 pred_pose = prev_pose * odom.simulatorTransf;

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
    std::cerr << "FUCK. Base pose in AddOdometry doesn't exist yet.!\n";
  }
}

int main()
{
  // TODO simulate different sensor offset
  // simulate a robot observing landmarks while travelling on a grid
  SE2 sensorOffsetTransf(0.2, 0.1, -0.1);
  int numNodes = 300;
  Simulator simulator;
  simulator.simulate(numNodes, sensorOffsetTransf);

  BearingSlam bs;

  /*********************************************************************************
   * creating the optimization problem
   ********************************************************************************/

  typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  // allocating the optimizer
  SparseOptimizer optimizer;
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);

  optimizer.setAlgorithm(solver);

  // add the parameter representing the sensor offset
  ParameterSE2Offset* sensorOffset = new ParameterSE2Offset;
  sensorOffset->setOffset(sensorOffsetTransf);
  sensorOffset->setId(0);
  optimizer.addParameter(sensorOffset);

  // adding the odometry to the optimizer
  // first adding all the vertices
  cerr << "Optimization: Adding robot poses ... ";
  for (size_t i = 0; i < simulator.poses().size(); ++i) {
    const Simulator::GridPose& p = simulator.poses()[i];
    const SE2& t = p.simulatorPose; 
    VertexSE2* robot =  new VertexSE2;
    robot->setId(p.id);
    robot->setEstimate(t);
    optimizer.addVertex(robot);
  }
  cerr << "done." << endl;

  // second add the odometry constraints
  cerr << "Optimization: Adding odometry measurements ... ";
  for (size_t i = 0; i < simulator.odometry().size(); ++i) {
    const Simulator::GridEdge& simEdge = simulator.odometry()[i];

    EdgeSE2* odometry = new EdgeSE2;
    odometry->vertices()[0] = optimizer.vertex(simEdge.from);
    odometry->vertices()[1] = optimizer.vertex(simEdge.to);
    odometry->setMeasurement(simEdge.simulatorTransf);
    odometry->setInformation(simEdge.information);
    optimizer.addEdge(odometry);
  }
  cerr << "done." << endl;

  // add the landmark observations
  cerr << "Optimization: add landmark vertices ... ";
  for (size_t i = 0; i < simulator.landmarks().size(); ++i) {
    const Simulator::Landmark& l = simulator.landmarks()[i];
    VertexPointXY* landmark = new VertexPointXY;
    landmark->setId(l.id);
    landmark->setEstimate(l.simulatedPose);
    optimizer.addVertex(landmark);
  }
  cerr << "done." << endl;

  cerr << "Optimization: add landmark observations ... ";
  for (size_t i = 0; i < simulator.landmarkObservations().size(); ++i) {
    const Simulator::LandmarkEdge& simEdge = simulator.landmarkObservations()[i];
    EdgeSE2PointXY* landmarkObservation =  new EdgeSE2PointXY;
    landmarkObservation->vertices()[0] = optimizer.vertex(simEdge.from);
    landmarkObservation->vertices()[1] = optimizer.vertex(simEdge.to);
    landmarkObservation->setMeasurement(simEdge.simulatorMeas);
    landmarkObservation->setInformation(simEdge.information);
    landmarkObservation->setParameterId(0, sensorOffset->id());
    optimizer.addEdge(landmarkObservation);
  }
  cerr << "done." << endl;


  /*********************************************************************************
   * optimization
   ********************************************************************************/

  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(optimizer.vertex(0));
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  cerr << "Optimizing" << endl;
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  cerr << "done." << endl;

  // freeing the graph memory
  optimizer.clear();

  // destroy all the singletons
  Factory::destroy();
  OptimizationAlgorithmFactory::destroy();
  // HyperGraphActionLibrary::destroy();

  return 0;
}
