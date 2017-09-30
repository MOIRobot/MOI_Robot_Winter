#include <global_planner/planner_core.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <winter_globalplanner/move_base_G.h>

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

namespace global_planner {

class PlannerWithCostmap : public GlobalPlanner {
    public:
        PlannerWithCostmap(string name, Costmap2DROS* cmap);
        bool makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& resp);

    private:
        void poseCallback(const rm::PoseStamped::ConstPtr& goal);
        Costmap2DROS* cmap_;
        ros::ServiceServer make_plan_service_;
        ros::Subscriber pose_sub_;
        rm::PoseStamped mgoal;
};

//服务器处理函数　　　此函数是与其他服务进行交互的函数
bool PlannerWithCostmap::makePlanService(navfn::MakeNavPlan::Request& req, navfn::MakeNavPlan::Response& resp) {
    //初始化一个路径
    vector<PoseStamped> path;
	req.start.header.frame_id = "/map";
    req.goal.header.frame_id = "/map";
    //开始进行路径规划　//开始做一个路径规划的计划 这个是继承父类GlobalPlanner的一个方法 定义在planner_core.cpp里
    bool success = makePlan(req.start, req.goal, path);
    resp.plan_found = success;
    if (success) {
        resp.path = path;
        //将路径规划的结果返回给ｓｅｒｖｉｃｅｃｌｉｅｎｔ请求方
    }

    return true;
}

//当在ｇｏａｌ话题上接收到PoseStamped消息时的处理函数
void PlannerWithCostmap::poseCallback(const rm::PoseStamped::ConstPtr& goal) {
    //初始化一个全局坐标
    tf::Stamped<tf::Pose> global_pose;
    //将机器人坐标放入到ｇｌｏｂａｌ＿ｐｏｓｅ里
    cmap_->getRobotPose(global_pose);
    //初始化一个路径
    vector<PoseStamped> path;
    //初始化一个起始点
    rm::PoseStamped start;

    start.header.stamp = global_pose.stamp_;
    start.header.frame_id = global_pose.frame_id_;
    start.pose.position.x = global_pose.getOrigin().x();
    start.pose.position.y = global_pose.getOrigin().y();
    start.pose.position.z = global_pose.getOrigin().z();
    start.pose.orientation.x = global_pose.getRotation().x();
    start.pose.orientation.y = global_pose.getRotation().y();
    start.pose.orientation.z = global_pose.getRotation().z();
    start.pose.orientation.w = global_pose.getRotation().w();


    mgoal.header.stamp = goal->header.stamp;
    mgoal.header.frame_id = "/map";
    mgoal.pose.position.x = goal->pose.position.x;
    mgoal.pose.position.y = goal->pose.position.y;
    mgoal.pose.position.z = goal->pose.position.z;
    mgoal.pose.orientation.x = goal->pose.orientation.x ;
    mgoal.pose.orientation.y = goal->pose.orientation.y;
    mgoal.pose.orientation.z = goal->pose.orientation.z;
    mgoal.pose.orientation.w = goal->pose.orientation.w;

     //开始做一个路径规划的计划 这个是继承父类GlobalPlanner的一个方法 定义在planner_core.cpp里
    ROS_INFO("I have got a Goal");
    ROS_INFO("the start position flame id is %s ",  start.header.frame_id.c_str());
    ROS_INFO("the start position.x is %f position.y is %f", start.pose.position.x, start.pose.position.y);
    ROS_INFO("the goal position flame id is %s ",  mgoal.header.frame_id.c_str() );
    ROS_INFO("the goal position.x is %f position.y is %f", goal->pose.position.x, goal->pose.position.y);

    makePlan(start, mgoal, path);
}

//构造函数　初始化PlannerWithCostmap对象
PlannerWithCostmap::PlannerWithCostmap(string name, Costmap2DROS* cmap) :
        GlobalPlanner(name, cmap->getCostmap(), cmap->getGlobalFrameID()) {
    /*
    PlannerWithCostmap继承于GlobalPlanner 此处也会接着初始化GlobalPlanner父类
    初始化函数在planner_core.cpp里 所以此处就相当于实例化了一个GlobalPlanner类
	*/
	//初始化一个全局规划器　其名字为ｎａｍｅ
    // getcostmap Return a pointer to the "master" costmap which receives updates from all the layers. 
    ros::NodeHandle private_nh("~");
    cmap_ = cmap;
    //发布ｍａｋｅｐｌａｎ服务　等待处理请求
    make_plan_service_ = private_nh.advertiseService("planservice", &PlannerWithCostmap::makePlanService, this);
    //订阅　ｇｏａｌ话题　
    pose_sub_ = private_nh.subscribe<rm::PoseStamped>("/move_base_simple/goal", 1, &PlannerWithCostmap::poseCallback, this);
}

} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_planner");

    //  a costmap_2d::Costmap2DROS object
    //创建一个lcr　costmap_2d::Costmap2DROS对象
    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS lcr("costmap", tf);

    //将lcr　costmap_2d::Costmap2DROS对象传入全局规划器 PlannerWithCostmap
    global_planner::PlannerWithCostmap pppp("planner", &lcr);
    
    /*
     costmap_2d::Costmap2D *costmap_ =   lcr.getCostmap();
	 //机器人的footprint
    std::vector<geometry_msgs::Point> footprint_spec=lcr.getRobotFootprint ();
	agv::LocalMoveBase mMove(costmap_,footprint_spec);
	*/
	 std::vector<geometry_msgs::Point> footprint_spec=lcr.getRobotFootprint ();
	agv::LocalMoveBase mMove(&lcr,&tf,footprint_spec);
	ros::spin();
    return 0;
}

