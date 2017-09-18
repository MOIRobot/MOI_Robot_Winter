#ifndef MCLEAR_COSTMAP_RECOVERY_H_
#define MCLEAR_COSTMAP_RECOVERY_H_
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>

namespace agv{
  /**
   * @class ClearCostmapRecovery
   * @brief A recovery behavior that reverts the navigation stack's costmaps to the static map outside of a user-specified region.
   */
  class MClearCostmapRecovery : public nav_core::RecoveryBehavior {
    public:
      /**
       * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
       * @param  
       * @return 
       */
      MClearCostmapRecovery();

      /**
       * @brief  Initialization function for the MClearCostmapRecovery recovery behavior
       * @param tf A pointer to a transform listener
       * @param global_costmap A pointer to the global_costmap used by the navigation stack 
       * @param local_costmap A pointer to the local_costmap used by the navigation stack 
       */
      void initialize(std::string name, tf::TransformListener* tf, 
          costmap_2d::Costmap2DROS* global_costmap);
	void initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);
      /**
       * @brief  Run the MClearCostmapRecovery recovery behavior. Reverts the
       * costmap to the static map outside of a user-specified window and
       * clears unknown space around the robot.
       */
      void runBehavior();

    private:
      void clear(costmap_2d::Costmap2DROS* costmap);      
      void clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, double pose_x, double pose_y);
      costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
      std::string name_;
      tf::TransformListener* tf_;
      bool initialized_;
      double reset_distance_;
      std::set<std::string> clearable_layers_; ///< Layer names which will be cleared.
  };
};
#endif  
