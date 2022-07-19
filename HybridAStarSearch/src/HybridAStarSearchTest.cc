#include "planner/HybridAStarSearchMap.h"

class ObstacleAnalyzer {
 public:
  ObstacleAnalyzer() = default;
  //初始化
  void Init() {
    //初始化路径搜索类 设置分辨率，边界
    search_map = std::make_shared<HybridAStarSearchMap>(
        "/home/ubuntu/catkin_ws/src/motion_planning/conf/"
        "motion_planning.pb.conf.sample");
    // 进行分辨率设置
    search_map->SetXYResolution(0.3);
    search_map->SetPhiResolution(0.2);
    // search_map->SetBounds(-5, 5, -3, 20);
    search_map->SetBounds(-10, 10, -10, 10);
    // 订阅障碍物以及目标点
    sub_obstacle_ = nh_.subscribe("/obstacles", 1,
                                  &ObstacleAnalyzer::ObstacleHandler, this);
    sub_rviz_goal_ = nh_.subscribe("/move_base_simple/goal", 1,
                                   &ObstacleAnalyzer::GoalHandler, this);
  }

  //获取rviz中人为设定的目标点
  void GoalHandler(geometry_msgs::PoseStamped msg) {
    std::cout << "x: " << msg.pose.orientation.x
              << " y: " << msg.pose.orientation.y
              << " z: " << msg.pose.orientation.z
              << " w: " << msg.pose.orientation.w << std::endl;
    dest_x = msg.pose.position.x;
    dest_y = msg.pose.position.y;
    // 航向处理有点不太清楚 
    double phi_cos = 2 * std::acos(msg.pose.orientation.w);

    dest_phi = msg.pose.orientation.z > 0 ? phi_cos : -phi_cos;

    

    std::cout << "x: " << dest_x << " y: " << dest_y << " phi: " << dest_phi << std::endl;
  }

  // 回调障碍物函数,获取实时的障碍物。并展开路径搜索
  void ObstacleHandler(costmap_converter::ObstacleArrayMsgConstPtr msg_ptr) 
  {
    if(last_dest_x!=dest_x &&last_dest_y!=dest_y )
    {
          // 数据重置
        search_map->Reset();
        ros::Time start = ros::Time::now();
        // 将障碍物添加到栅格图中
        search_map->AddObstacleArrayPtr(msg_ptr);

         std::cout<<"x:"<<dest_x<<"y:"<<dest_y<<",phi:"<<dest_phi<<std::endl;
        // 设置起点和目标点
        search_map->SetStartPoint(0, 0, 3.14/2.0);
        search_map->SetEndPoint(dest_x, dest_y, dest_phi);

        std::vector<double> bound=search_map->GetBound();
        std::cout<<"x:"<<bound[0]<<"x max:"<<bound[1]<<",y min:"<<bound[2]<<",y max:"<<bound[3]<<std::endl;
         // 产生启发式地图
        if (!search_map->GenerateHeuristicMap()) {
          ROS_ERROR("Map Initialization Failure!");
          return;
        }
        std::cout << "time for map generation: " << ros::Time::now() - start << std::endl;
        // search_map->PlotHeuristicMap();

        // 路径搜索
        start = ros::Time::now();
        search_map->Search();
        std::cout << "time for search: " << ros::Time::now() - start << std::endl;
    }

      
    last_dest_x=dest_x;
    last_dest_y=dest_y;
    last_dest_phi=dest_phi;
   
    search_map->PlotDebugMap();
    search_map->PlotTrajectory();
  }

 private:
  double dest_x = 0, dest_y = 0, dest_phi = 0;

  double last_dest_x = 0, last_dest_y = 0, last_dest_phi = 0;

  std::shared_ptr<HybridAStarSearchMap> search_map;

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_obstacle_, sub_rviz_goal_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "Astar");
  ros::NodeHandle nh;

  ObstacleAnalyzer obstacle_analyzer;
  obstacle_analyzer.Init();

  ros::spin();
  return 0;
}