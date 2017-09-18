#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <winter_globalplanner/move_base_G.h>
						
 int main(int argc, char** argv) 
 {
    //顺序不可以改变 init和node
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle n;
	ros::Rate loop_rate(0.5);
	
    //创建一个lcr　costmap_2d::Costmap2DROS对象
    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS lcr("localcostmap", tf);

    costmap_2d::Costmap2D *costmap_ =   lcr.getCostmap();
	 //机器人的footprint
    std::vector<geometry_msgs::Point> footprint_spec=lcr.getRobotFootprint ();
	AGV::LocalMoveBase mMove(costmap_,footprint_spec);
	ros::spin();
    return 0;
}
