#include <clear_costmap_recovery_gao/clear_costmap_recovery_gao.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(clear_costmap_recovery_gao, ClearCostmapRecoveryGao, clear_costmap_recovery_gao::ClearCostmapRecoveryGao, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace clear_costmap_recovery_gao {
ClearCostmapRecoveryGao::ClearCostmapRecoveryGao(): global_costmap_(NULL), 
  tf_(NULL), initialized_(false) {} 

void ClearCostmapRecoveryGao::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_    = local_costmap;
    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);

    private_nh.param("reset_distance", reset_distance_, 3.0);
    
    std::vector<std::string> clearable_layers_default, clearable_layers;
    private_nh.param("layer_names", clearable_layers, clearable_layers_default);

    for(unsigned i=0; i < clearable_layers.size(); i++) {
        ROS_INFO("Recovery behavior will clear layer %s", clearable_layers[i].c_str());
        clearable_layers_.insert(clearable_layers[i]);
    }


    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void ClearCostmapRecoveryGao::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ != NULL){
    ROS_WARN("Clearing costmap to unstuck robot (%fm).", reset_distance_);
    clear(global_costmap_);
  }
  else
  {
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  
  if(local_costmap_ != NULL){
    ROS_WARN("Clearing costmap to unstuck robot (%fm).", reset_distance_);
    clear(local_costmap_);
  }
  else
  {
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  
  
}

//选择要清除的层 以及要清除的多少距离以外的地图
void ClearCostmapRecoveryGao::clearOnelayer(const char* name, double distance_){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  reset_distance_=distance_;
  if(global_costmap_ != NULL){
    clear(global_costmap_,name);
    ROS_WARN("Clearing costmap to unstuck robot (%fm).", reset_distance_);
   }
   else
   {
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    }
    
    if(local_costmap_ != NULL){
    clear(local_costmap_,name);
    ROS_WARN("Clearing costmap to unstuck robot (%fm).", reset_distance_);
   }
   else
   {
    ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
    }
    
    
  
}

void ClearCostmapRecoveryGao::clear(costmap_2d::Costmap2DROS* costmap){
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

  tf::Stamped<tf::Pose> pose;

  if(!costmap->getRobotPose(pose)){
    ROS_ERROR("Cannot clear map because pose cannot be retrieved");
    return;
  }

  double x = pose.getOrigin().x();
  double y = pose.getOrigin().y();

  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
    boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
    
    //获取costmap plugin 的名称
    std::string name = plugin->getName();
    int slash = name.rfind('/');
    if( slash != std::string::npos ){
        name = name.substr(slash+1);
    }

    if(clearable_layers_.count(name)!=0){
		
		//对比是不是我们添加的要清除的层 如果是 则进行以下清除操作
      boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
      costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
      clearMap(costmap, x, y);
    }
  }
}
void ClearCostmapRecoveryGao::clear(costmap_2d::Costmap2DROS* costmap,const char* layer_name){
  std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = costmap->getLayeredCostmap()->getPlugins();

  tf::Stamped<tf::Pose> pose;

  if(!costmap->getRobotPose(pose)){
    ROS_ERROR("Cannot clear map because pose cannot be retrieved");
    return;
  }

  double x = pose.getOrigin().x();
  double y = pose.getOrigin().y();

  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {
    boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
    
    //获取costmap plugin 的名称
    std::string name = plugin->getName();
    int slash = name.rfind('/');
    if( slash != std::string::npos ){
        name = name.substr(slash+1);
    }
   const char *p=name.data();
	//清除指定的层
    if(std::strcmp(p,layer_name)==0){
		
		//对比是不是我们添加的要清除的层 如果是 则进行以下清除操作
      boost::shared_ptr<costmap_2d::CostmapLayer> costmap;
      costmap = boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
      clearMap(costmap, x, y);
      return ;
    }
  }
}

void ClearCostmapRecoveryGao::clearMap(boost::shared_ptr<costmap_2d::CostmapLayer> costmap, 
                                        double pose_x, double pose_y){
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
 
  double start_point_x = pose_x - reset_distance_ / 2;
  double start_point_y = pose_y - reset_distance_ / 2;
  double end_point_x = start_point_x + reset_distance_;
  double end_point_y = start_point_y + reset_distance_;

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapNoBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapNoBounds(end_point_x, end_point_y, end_x, end_y);

  unsigned char* grid = costmap->getCharMap();
  for(int x=0; x<(int)costmap->getSizeInCellsX(); x++){
    bool xrange = x>start_x && x<end_x;
                   
    for(int y=0; y<(int)costmap->getSizeInCellsY(); y++){
      if(xrange && y>start_y && y<end_y)
        continue;
      int index = costmap->getIndex(x,y);
      if(grid[index]!=NO_INFORMATION){
        grid[index] = NO_INFORMATION;
      }
    }
  }

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
  return;
}

};
