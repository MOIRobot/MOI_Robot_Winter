/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <base_local_planner/trajectory_planner.h>
#include <costmap_2d/footprint.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>



#include <boost/algorithm/string.hpp>

#include <ros/console.h>

//for computing path distance
#include <queue>

using namespace std;
using namespace costmap_2d;

namespace base_local_planner{

  void Winter_TrajectoryPlanner::mreconfigure(BaseLocalPlannerConfig &cfg)
  {
      BaseLocalPlannerConfig config(cfg);

      boost::mutex::scoped_lock l(configuration_mutex_);

      acc_lim_x_ = config.acc_lim_x;
      acc_lim_y_ = config.acc_lim_y;
      acc_lim_theta_ = config.acc_lim_theta;

      max_vel_x_ = config.max_vel_x;
      min_vel_x_ = config.min_vel_x;
      
      max_vel_th_ = config.max_vel_theta;
      min_vel_th_ = config.min_vel_theta;
      min_in_place_vel_th_ = config.min_in_place_vel_theta;

      sim_time_ = config.sim_time;
      sim_granularity_ = config.sim_granularity;
      angular_sim_granularity_ = config.angular_sim_granularity;

      pdist_scale_ = config.pdist_scale;
      gdist_scale_ = config.gdist_scale;
      occdist_scale_ = config.occdist_scale;

      if (meter_scoring_) {
        //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
        double resolution = costmap_.getResolution();
        gdist_scale_ *= resolution;
        pdist_scale_ *= resolution;
        occdist_scale_ *= resolution;
      }

      oscillation_reset_dist_ = config.oscillation_reset_dist;
      escape_reset_dist_ = config.escape_reset_dist;
      escape_reset_theta_ = config.escape_reset_theta;

      vx_samples_ = config.vx_samples;
      vtheta_samples_ = config.vtheta_samples;

      if (vx_samples_ <= 0) {
          config.vx_samples = 1;
          vx_samples_ = config.vx_samples;
          ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      }
      if(vtheta_samples_ <= 0) {
          config.vtheta_samples = 1;
          vtheta_samples_ = config.vtheta_samples;
          ROS_WARN("You've specified that you don't want any samples in the theta dimension. We'll at least assume that you want to sample one value... so we're going to set vtheta_samples to 1 instead");
      }

      heading_lookahead_ = config.heading_lookahead;

      holonomic_robot_ = config.holonomic_robot;
      
      backup_vel_ = config.escape_vel;

      dwa_ = config.dwa;

      heading_scoring_ = config.heading_scoring;
      heading_scoring_timestep_ = config.heading_scoring_timestep;

      simple_attractor_ = config.simple_attractor;

      //y-vels
      string y_string = config.y_vels;
      vector<string> y_strs;
      boost::split(y_strs, y_string, boost::is_any_of(", "), boost::token_compress_on);

      vector<double>y_vels;
      for(vector<string>::iterator it=y_strs.begin(); it != y_strs.end(); ++it) {
          istringstream iss(*it);
          double temp;
          iss >> temp;
          y_vels.push_back(temp);
          //ROS_INFO("Adding y_vel: %e", temp);
      }

      y_vels_ = y_vels;
      
  }

  Winter_TrajectoryPlanner::Winter_TrajectoryPlanner(WorldModel& world_model,
      const Costmap2D& costmap,
      std::vector<geometry_msgs::Point> footprint_spec,
      double acc_lim_x, double acc_lim_y, double acc_lim_theta,
      double sim_time, double sim_granularity,
      int vx_samples, int vtheta_samples,
      double pdist_scale, double gdist_scale, double occdist_scale,
      double heading_lookahead, double oscillation_reset_dist,
      double escape_reset_dist, double escape_reset_theta,
      bool holonomic_robot,
      double max_vel_x, double min_vel_x,
      double max_vel_th, double min_vel_th, double min_in_place_vel_th,
      double backup_vel,
      bool dwa, bool heading_scoring, double heading_scoring_timestep, bool meter_scoring, bool simple_attractor,
      vector<double> y_vels, double stop_time_buffer, double sim_period, double angular_sim_granularity)
    : path_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
      goal_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
      costmap_(costmap),
    world_model_(world_model), footprint_spec_(footprint_spec),
    sim_time_(sim_time), sim_granularity_(sim_granularity), angular_sim_granularity_(angular_sim_granularity),
    vx_samples_(vx_samples), vtheta_samples_(vtheta_samples),
    pdist_scale_(pdist_scale), gdist_scale_(gdist_scale), occdist_scale_(occdist_scale),
    acc_lim_x_(acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(acc_lim_theta),
    prev_x_(0), prev_y_(0), escape_x_(0), escape_y_(0), escape_theta_(0), heading_lookahead_(heading_lookahead),
    oscillation_reset_dist_(oscillation_reset_dist), escape_reset_dist_(escape_reset_dist),
    escape_reset_theta_(escape_reset_theta), holonomic_robot_(holonomic_robot),
    max_vel_x_(max_vel_x), min_vel_x_(min_vel_x),
    max_vel_th_(max_vel_th), min_vel_th_(min_vel_th), min_in_place_vel_th_(min_in_place_vel_th),
    backup_vel_(backup_vel),
    dwa_(dwa), heading_scoring_(heading_scoring), heading_scoring_timestep_(heading_scoring_timestep),
    simple_attractor_(simple_attractor), y_vels_(y_vels), stop_time_buffer_(stop_time_buffer), sim_period_(sim_period)
  {
    //the robot is not stuck to begin with
    stuck_left = false;
    stuck_right = false;
    stuck_left_strafe = false;
    stuck_right_strafe = false;
    rotating_left = false;
    rotating_right = false;
    strafe_left = false;
    strafe_right = false;

    escaping_ = false;
    final_goal_position_valid_ = false;

	if(vx_samples_%2!=0) vx_samples_+=1;
    if(vtheta_samples_%2!=0) vtheta_samples_+=1;
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);
  }

  Winter_TrajectoryPlanner::~Winter_TrajectoryPlanner(){}

  bool Winter_TrajectoryPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
    MapCell cell = path_map_(cx, cy);
    MapCell goal_cell = goal_map_(cx, cy);
    if (cell.within_robot) {
        return false;
    }
    occ_cost = costmap_.getCost(cx, cy);
    if (cell.target_dist == path_map_.obstacleCosts() ||
        cell.target_dist == path_map_.unreachableCellCosts() ||
        occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
    }
    path_cost = cell.target_dist;
    goal_cost = goal_cell.target_dist;
    total_cost = pdist_scale_ * path_cost + gdist_scale_ * goal_cost + occdist_scale_ * occ_cost;
    return true;
  }

  /**
   * create and score a trajectory given the current pose of the robot and selected velocities
   */
  void Winter_TrajectoryPlanner::mgenerateTrajectory(
      double x, double y, double theta,
      double vx, double vy, double vtheta,
      double vx_samp, double vy_samp, double vtheta_samp,
      double acc_x, double acc_y, double acc_theta,
      double impossible_cost,
      Trajectory& traj) {

	//ROS_INFO("mgenerateTrajectory ");
    // make sure the configuration doesn't change mid run
    boost::mutex::scoped_lock l(configuration_mutex_);

	//获取当前位置与朝向 
    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    double vx_i, vy_i, vtheta_i;
	//获取当前机器人速度
    vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

    //compute the magnitude of the velocities
    //计算机器人的和速度
    double vmag = hypot(vx_samp, vy_samp);

    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps;
    /*
     *  heading_scoring: 通过机器人航向计算还是通过路径计算距离，默认false     heading_scoring_timestep: 航向计算距离时，沿着模拟轨迹向前看的时间，默认0.8
     * */
     //从这里求得采样点数目 在采样时间内机器人运动的距离除以采样精度
    if(!heading_scoring_) {
      num_steps = int(max((vmag * sim_time_) / sim_granularity_, fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);
    } else {
      num_steps = int(sim_time_ / sim_granularity_ + 0.5);
    }

    //we at least want to take one step... even if we won't move, we want to score our current position
    if(num_steps == 0) {
      num_steps = 1;
    }

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
    double heading_diff = 0.0;
    //开始循环 这么多的步数
    for(int i = 0; i < num_steps; ++i){
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the know map
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        traj.cost_ = -1.0;
        return;
      }

      //check the point on the trajectory for legality
      double footprint_cost = footprintCost(x_i, y_i, theta_i);

      //if the footprint hits an obstacle this trajectory is invalid
      if(footprint_cost < 0){
        traj.cost_ = -1.0;
        return;
      }

      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));
	
        bool update_path_and_goal_distances = true;

        // with heading scoring, we take into account heading diff, and also only score
        // path and goal distance for one point of the trajectory
        if (heading_scoring_) {
          if (time >= heading_scoring_timestep_ && time < heading_scoring_timestep_ + dt) {
            heading_diff = headingDiff(cell_x, cell_y, x_i, y_i, theta_i);
          } else {
            update_path_and_goal_distances = false;
          }
        }

        if (update_path_and_goal_distances) {
          //update path and goal distances
          path_dist = path_map_(cell_x, cell_y).target_dist;
          goal_dist = goal_map_(cell_x, cell_y).target_dist;

          //if a point on this trajectory has no clear path to goal it is invalid
          if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
//            ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f",
//                goal_dist, path_dist, impossible_cost);
            traj.cost_ = -2.0;
            return;
          }
        }


      //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);

      //calculate velocities
      vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
      vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
      vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

      //calculate positions
      x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
      y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
      theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

      //increment time
      time += dt;
    } // end for i < numsteps

    //ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
    double cost = -1.0;
    //采用　距离　方向　同时打分
    //ROS_INFO("gdist_scale_: %f, pdist_scale_: %.2f, occdist_scale_: %.2f", gdist_scale_, pdist_scale_, occdist_scale_);
    cost = occdist_scale_ * occ_cost + pdist_scale_ * path_dist + 0.3 * heading_diff + goal_dist * gdist_scale_;
    //cost = occdist_scale_ * occ_cost + pdist_scale_ * path_dist + 0.3 * heading_diff;
    //ROS_INFO("cost %f",cost);
    traj.cost_ = cost;
  }

  double Winter_TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
    double heading_diff = DBL_MAX;
    unsigned int goal_cell_x, goal_cell_y;
    const double v2_x = cos(heading);
    const double v2_y = sin(heading);

    //find a clear line of sight from the robot's cell to a point on the path
    for (int i = global_plan_.size() - 1; i >=0; --i) {
      if (costmap_.worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y)) {
        if (lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0) {
          double gx, gy;
          costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
          double v1_x = gx - x;
          double v1_y = gy - y;

          double perp_dot = v1_x * v2_y - v1_y * v2_x;
          double dot = v1_x * v2_x + v1_y * v2_y;

          //get the signed angle
          double vector_angle = atan2(perp_dot, dot);

          heading_diff = fabs(vector_angle);
          return heading_diff;
        }
      }
    }
    return heading_diff;
  }

  //calculate the cost of a ray-traced line
  double Winter_TrajectoryPlanner::lineCost(int x0, int x1,
      int y0, int y1){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    double line_cost = 0.0;
    double point_cost = -1.0;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    } else {                      // There is at least one y-value for every x-value
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
      point_cost = pointCost(x, y); //Score the current point

      if (point_cost < 0) {
        return -1;
      }

      if (line_cost < point_cost) {
        line_cost = point_cost;
      }

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den) {           // Check if numerator >= denominator
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }

    return line_cost;
  }

  double Winter_TrajectoryPlanner::pointCost(int x, int y){
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
      return -1;
    }

    return cost;
  }

  void Winter_TrajectoryPlanner::mupdatePlan(const vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists){
    global_plan_.resize(new_plan.size());
    for(unsigned int i = 0; i < new_plan.size(); ++i){
      global_plan_[i] = new_plan[i];
    }

    if( global_plan_.size() > 0 ){
      geometry_msgs::PoseStamped& final_goal_pose = global_plan_[ global_plan_.size() - 1 ];
      final_goal_x_ = final_goal_pose.pose.position.x;
      final_goal_y_ = final_goal_pose.pose.position.y;
      final_goal_position_valid_ = true;
    } else {
      final_goal_position_valid_ = false;
    }

    if (compute_dists) {
      //reset the map for new operations
      path_map_.resetPathDist();
      goal_map_.resetPathDist();

      //make sure that we update our path based on the global plan and compute costs
      path_map_.setTargetCells(costmap_, global_plan_);
      goal_map_.setLocalGoal(costmap_, global_plan_);
      ROS_DEBUG("Path/Goal distance computed");
    }
  }

bool Winter_TrajectoryPlanner::mcheckTrajectory(double x, double y, double theta, double vx, double vy,
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
    Trajectory t;

    double cost = mscoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);

    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp, vtheta_samp, cost);

    //otherwise the check fails
    return false;
  }

  double Winter_TrajectoryPlanner::mscoreTrajectory(double x, double y, double theta, double vx, double vy,
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp) {
    Trajectory t;
    double impossible_cost = path_map_.obstacleCosts();
    mgenerateTrajectory(x, y, theta,
                       vx, vy, vtheta,
                       vx_samp, vy_samp, vtheta_samp,
                       acc_lim_x_, acc_lim_y_, acc_lim_theta_,
                       impossible_cost, t);

    // return the cost.
    return double( t.cost_ );
  }

  /*
   * create the trajectories we wish to score
   */
  Trajectory Winter_TrajectoryPlanner::mcreateTrajectories(double x, double y, double theta,
      double vx, double vy, double vtheta,
      double acc_x, double acc_y, double acc_theta) {
    //compute feasible velocity limits in robot space
    double max_vel_x = max_vel_x_, max_vel_theta;
    double min_vel_x, min_vel_theta;
	
    if( final_goal_position_valid_ ){
      double final_goal_dist = hypot( final_goal_x_ - x, final_goal_y_ - y );
      max_vel_x = min( max_vel_x, final_goal_dist / sim_time_ );
    }

    //should we use the dynamic window approach　这里使用ＤＷＡ 直接删去原来不是的ｄｗａ代码
      max_vel_x = max(min(max_vel_x, vx + acc_x * sim_period_), min_vel_x_);
      min_vel_x = max(min_vel_x_, vx - acc_x * sim_period_);

      max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_period_);
      min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_period_);

   /**tag First ************************************************************************************************/
   /**************************************************************************************************/
    /*we want to sample the velocity space regularly　　这里将vx_samples_　还有vtheta_samples_　定义为这个最大最小速度中间有多少段速度
     * |____|__|__|___| 始终为偶数　中间的为当前值
    */
    
    double dvx = (max_vel_x - min_vel_x) / vx_samples_ ;
    
    //角度间隔均匀增加
    double dvtheta = (max_vel_theta - min_vel_theta) /vtheta_samples_ ;

    double vx_samp = min_vel_x;
    double vtheta_samp = min_vel_theta;
	double vy_samp = 0.0;
	
	//let's try to rotate toward open space
	double heading_dist = DBL_MAX;
	
    //keep track of the best trajectory seen so far
    Trajectory* best_traj = &traj_one;
    best_traj->cost_ = -1.0;

    Trajectory* comp_traj = &traj_two;
    comp_traj->cost_ = -1.0;

	Trajectory* swap = NULL;
	//ROS_INFO("move trajectoryDWA ");
	double impossible_cost = path_map_.obstacleCosts();
	//向前的动态规划路径
   trajectoryDWA(best_traj,comp_traj ,
							    x,  y,  theta, vx,  vy,  vtheta,acc_x, acc_y, acc_theta,
								min_vel_x,max_vel_x,min_vel_theta,max_vel_theta,
								dvx,dvtheta);
	//原地旋转路径
	//ROS_INFO("trajectoryRotate ");
  /*trajectoryRotate(best_traj,comp_traj ,
									x,  y,  theta, vx,  vy,  vtheta,acc_x,  acc_y,  acc_theta,
									min_vel_x,min_vel_theta,
									dvx,dvtheta,heading_dist); */
    //do we have a legal trajectory
    if (best_traj->cost_ >= 0) {
      if (!(best_traj->xv_ > 0)) {
        if (best_traj->thetav_ < 0) {
          if (rotating_right){
            stuck_right = true;
          }
          rotating_left = true;
        } else if(best_traj->thetav_ > 0) {
          if(rotating_left){
            stuck_left = true;
          }
          rotating_right = true;
        } else if(best_traj->yv_ > 0) {
          if(strafe_right){
            stuck_right_strafe = true;
          }
          strafe_left = true;
        } else if(best_traj->yv_ < 0) {
          if(strafe_left){
            stuck_left_strafe = true;
          }
          strafe_right = true;
        }

        //set the position we must move a certain distance away from
        prev_x_ = x;
        prev_y_ = y;

      }

      double dist = hypot(x - prev_x_, y - prev_y_);
      if(dist > oscillation_reset_dist_) {
        rotating_left = false;
        rotating_right = false;
        strafe_left = false;
        strafe_right = false;
        stuck_left = false;
        stuck_right = false;
        stuck_left_strafe = false;
        stuck_right_strafe = false;
      }

      dist = hypot(x - escape_x_, y - escape_y_);
      if(dist > escape_reset_dist_ || fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
        escaping_ = false;
      }

      return *best_traj;
    }
	/**tag third*****************************************************************************************/
    /**************************************************************************************************/
	//向后退的路径
	//trajectoryMoveBack(best_traj,comp_traj ,x,  y,  theta,vx,  vy,  vtheta, acc_x,  acc_y,  acc_theta); 

    
    return *best_traj;
    

  }
  //given the current state of the robot, find a good trajectory
  Trajectory Winter_TrajectoryPlanner::mfindBestPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
      tf::Stamped<tf::Pose>& drive_velocities){
		  
    Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    //vel 机器人的速度
    Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));
	
    //reset the map for new operations
    path_map_.resetPathDist();
    goal_map_.resetPathDist();

    //temporarily remove obstacles that are within the footprint of the robot
    std::vector<base_local_planner::Position2DInt> footprint_list =
        footprint_helper_.getFootprintCells(
            pos,
            footprint_spec_,
            costmap_,
            true);

    //mark cells within the initial footprint of the robot
    for (unsigned int i = 0; i < footprint_list.size(); ++i) {
      path_map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
    }

    //make sure that we update our path based on the global plan and compute costs
    path_map_.setTargetCells(costmap_, global_plan_);
    //设置局部地图的边缘为local ｇｏａｌ
    goal_map_.setLocalGoal(costmap_, global_plan_);
    
    ROS_DEBUG("Path/Goal distance computed");
    //rollout trajectories and find the minimum cost one
    Trajectory best = mcreateTrajectories(pos[0], pos[1], pos[2],
        vel[0], vel[1], vel[2],
        acc_lim_x_, acc_lim_y_, acc_lim_theta_);
    
    if(best.cost_ < 0){
      drive_velocities.setIdentity();
    }
    else{
      tf::Vector3 start(best.xv_, best.yv_, 0);
      drive_velocities.setOrigin(start);
      tf::Matrix3x3 matrix;
      matrix.setRotation(tf::createQuaternionFromYaw(best.thetav_));
      drive_velocities.setBasis(matrix);
    }

    return best;
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double Winter_TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i){
    //check if the footprint is legal
    return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
  }
  bool Winter_TrajectoryPlanner::checkPath(tf::Stamped<tf::Pose> global_pose,std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
	int i;
	double cx=global_pose.getOrigin().x();
	double cy=global_pose.getOrigin().y();
	
	double lx ;
    double ly ;
    double x;
    double y;
    double angle;
    double yaw;
    bool flag=true;
    
	for(i=0;i<transformed_plan.size();i++)
	{
			x=transformed_plan[i].pose.position.x;
			y=transformed_plan[i].pose.position.y;
			
			if((abs(x-cx)<2)&&(abs(y-cy)<2))
				{
					yaw=tf::getYaw(transformed_plan[i].pose.orientation);
					//ROS_INFO("cs %f cy %f x %f,y %f cost %f",cx,cy,x,y,footprintCost(x,y,yaw));
					//if ((footprintCost(x,y,yaw)<0.0)||(footprintCost(x,y,yaw)>252)) return false;
					if ((footprintCost(x,y,yaw)<0.0)) return false;
				}
	}
	return flag;
}


  void Winter_TrajectoryPlanner::getLocalGoal(double& x, double& y){
    x = path_map_.goal_x_;
    y = path_map_.goal_y_;
  }
void Winter_TrajectoryPlanner::trajectoryDWA(
																					   Trajectory* &best_traj,Trajectory* &comp_traj ,
																					   double x, double y, double theta,
																					   double vx, double vy, double vtheta,
																					   double acc_x, double acc_y, double acc_theta,
																						double min_vel_x,double max_vel_x,double min_vel_theta,double max_vel_theta,
																						double dvx,double dvtheta) 
  {
		Trajectory* swap = NULL;

		double vx_samp = min_vel_x;
		double vtheta_samp = min_vel_theta;
		double vy_samp = 0.0;
		//集约度
		double scale=0.35;
		int half_vtheta_samples_=vtheta_samples_/2;
		double dvtheta_h=(max_vel_theta-vtheta)/half_vtheta_samples_;
		double dvtheta_l=(vtheta-min_vel_theta)/half_vtheta_samples_;
		//any cell with a cost greater than the size of the map is impossible
		double impossible_cost = path_map_.obstacleCosts();
   /**************************************************************************************************/
    //if we're performing an escape we won't allow moving forward
	//tag 如果是避让模式 只采样直线速度和原地旋转的速度
    if (!escaping_) {
      //loop through all x velocities
      for(int j = 0; j < vx_samples_; ++j) {

        vtheta_samp = min_vel_theta;
        //next sample all theta trajectories
        ////将角速度从最小到最大 采样各个角速度值 vtheta_samples_默认为偶数
        for(int i = 0; i < vtheta_samples_ ; ++i){
          //ROS_INFO("i %d vtheat %f vs %f ",i,vtheta,vtheta_samp);
          mgenerateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
          //if the new trajectory is better... let's take it
          if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){swap = best_traj;best_traj = comp_traj;comp_traj = swap;}
          
          if (i<half_vtheta_samples_) vtheta_samp=vtheta-(half_vtheta_samples_-i)*dvtheta_l*scale;
          else if(i==half_vtheta_samples_) vtheta_samp=vtheta;
          else
			{
				vtheta_samp=vtheta+(i+2-half_vtheta_samples_)*dvtheta_h*scale;
			}
        }
        
        //最后采样最大值的角速度
        vtheta_samp=max_vel_theta;
        mgenerateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){swap = best_traj;best_traj = comp_traj;comp_traj = swap;}
        //速度采样值增加
        vx_samp += dvx;
      }
    } // end if not escaping
  }
void Winter_TrajectoryPlanner::trajectoryRotate(
																					   Trajectory* &best_traj,Trajectory* &comp_traj ,
																					   double x, double y, double theta,
																					   double vx, double vy, double vtheta,
																					   double acc_x, double acc_y, double acc_theta,
																						double min_vel_x,double min_vel_theta,
																						double dvx,double dvtheta,double& heading_dist) 
{
	 
	Trajectory* swap = NULL;

	double vx_samp = 0.0;
	double vtheta_samp = min_vel_theta;
	double vy_samp = 0.0;
		
	//any cell with a cost greater than the size of the map is impossible
	double impossible_cost = path_map_.obstacleCosts();


	for(int i = 0; i < vtheta_samples_; ++i) {
      //enforce a minimum rotational velocity because the base can't handle small in-place rotations
      double vtheta_samp_limited = vtheta_samp > 0 ? max(vtheta_samp, min_in_place_vel_th_)
        : min(vtheta_samp, -1.0 * min_in_place_vel_th_);

      mgenerateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp_limited,
          acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

      //if the new trajectory is better... let's take it...
      //note if we can legally rotate in place we prefer to do that rather than move with y velocity
      if(comp_traj->cost_ >= 0
          && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0 || best_traj->yv_ != 0.0)
          && (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta)){
        double x_r, y_r, th_r;
        comp_traj->getEndpoint(x_r, y_r, th_r);
        x_r += heading_lookahead_ * cos(th_r);
        y_r += heading_lookahead_ * sin(th_r);
        unsigned int cell_x, cell_y;

        //make sure that we'll be looking at a legal cell
        if (costmap_.worldToMap(x_r, y_r, cell_x, cell_y)) {
          double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
          if (ahead_gdist < heading_dist) {
            //if we haven't already tried rotating left since we've moved forward
            if (vtheta_samp < 0 && !stuck_left) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
            //if we haven't already tried rotating right since we've moved forward
            else if(vtheta_samp > 0 && !stuck_right) {
              swap = best_traj;
              best_traj = comp_traj;
              comp_traj = swap;
              heading_dist = ahead_gdist;
            }
          }
        }
      }

      vtheta_samp += dvtheta;
    }
	}
void Winter_TrajectoryPlanner::trajectoryMoveBack(Trajectory* &best_traj,Trajectory* &comp_traj ,
																					double x, double y, double theta,
																					double vx, double vy, double vtheta,
																					double acc_x, double acc_y, double acc_theta) 
{

	//and finally, if we can't do anything else, we want to generate trajectories that move backwards slowly
	Trajectory* swap = NULL;

	double vx_samp = backup_vel_;
	double vtheta_samp = 0.0;
	double vy_samp = 0.0;
		
	//any cell with a cost greater than the size of the map is impossible
	double impossible_cost = path_map_.obstacleCosts();

	mgenerateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp,
    acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

		//if the new trajectory is better... let's take it
		/*本来就被屏蔽的
       if(comp_traj->cost_ >= 0 && (comp_traj->cost_ < best_traj->cost_ || best_traj->cost_ < 0)){
       swap = best_traj;
       best_traj = comp_traj;
       comp_traj = swap;
       }
       */

	//we'll allow moving backwards slowly even when the static map shows it as blocked
    swap = best_traj;
    best_traj = comp_traj;
    comp_traj = swap;

    double dist = hypot(x - prev_x_, y - prev_y_);
    if (dist > oscillation_reset_dist_) {
      rotating_left = false;
      rotating_right = false;
      strafe_left = false;
      strafe_right = false;
      stuck_left = false;
      stuck_right = false;
      stuck_left_strafe = false;
      stuck_right_strafe = false;
    }

    //only enter escape mode when the planner has given a valid goal point
    //从这里进入逃逸模式 只有planner 鬼畜有效目标点的时候
    if (!escaping_ && best_traj->cost_ > -2.0) {
      escape_x_ = x;
      escape_y_ = y;
      escape_theta_ = theta;
      escaping_ = true;
    }

    dist = hypot(x - escape_x_, y - escape_y_);

    if (dist > escape_reset_dist_ ||
        fabs(angles::shortest_angular_distance(escape_theta_, theta)) > escape_reset_theta_) {
      escaping_ = false;
    }


    //if the trajectory failed because the footprint hits something, we're still going to back up
    if(best_traj->cost_ == -1.0)
      best_traj->cost_ = 1.0;
      
}
void Winter_TrajectoryPlanner::trajectoryMoveX(Trajectory* &best_traj,Trajectory* &comp_traj ,
																					   double x, double y, double theta,
																					   double vx, double vy, double vtheta,
																					   double acc_x, double acc_y, double acc_theta,
																					   double max_vel_x,double max_vel_theta,
																					   double min_vel_x,double min_vel_theta,
																					   double dvx,double dvtheta) 
{
	/*
	double goal_distance=1.0;
	//以当前速度降速下来的距离
	double brake_distance=	vx*vx/(acc_x*2.0);
	double acc_time;//加速时间
	double constant_time;//匀速时间
	double brake_time;//减速时间
	
	double vx_i=vx;
	double vy_i=vy;
	double x_i=x;
	double y_i=y;
	
	bool changeFlag=false;
	//create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;
    traj.cost_ = 1.0;
    
	if(brake_distance>=goal_distance)
	{
		//如果机器人降速下来的距离大于要运动的距离机器人现在必须要减速运动 但是这个情况很少
	}
	else
	{
		
		while(true)
		{
		 double move_dist=hypot(x_i-x,y_i-y);
		 if((goal_distance-move_dist)>0.02)
		 {
			 //还没有到达目的地
			 
		 //get map coordinates of a point
		unsigned int cell_x, cell_y;
		//we don't want a path that goes off the know map
		if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
			traj.cost_ = -1.0;
			return;
		}
		//check the point on the trajectory for legality
		double footprint_cost = footprintCost(x_i, y_i, theta_i);
		//if the footprint hits an obstacle this trajectory is invalid
		if(footprint_cost < 0){
			traj.cost_ = -1.0;
			return;
      }
		//the point is legal... add it to the trajectory
		traj.addPoint(x_i, y_i, theta_i);
		
		brake_distance=vx_i*vx_i/(acc_x*2.0);
		if(!changeFlag)
		{
			if((goal_distance-move_dist)>brake_distance)
			{
				changeFlag=false;
			}
			else
			{
				changeFlag=true;
			}
			
		}
		if(!changeFlag)
		{//calculate velocities
			//向前仿真0.05秒
			vx_i=vx_i+acc_x*0.05;
			if(vx_i>=max_vel_x) vx_i=max_vel_x;
				
		}
		else
		{
			vx_i=vx_i-acc_x*0.05;
		}
		//calculate positions 速度 x y theat 时间
		x_i = computeNewXPosition(x_i, vx_i, vy_i, 0, 0.05);
		y_i = computeNewYPosition(y_i, vx_i, vy_i, 0, 0.05);
      
	}
	else
	{
		traj.addPoint(0, 0, 0);
		break;
	}
	}
	}
	traj.cost_=1.0;
	* */
} 
};


