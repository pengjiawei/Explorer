//============================================================================
// Name        : SeExplorer.cpp
// Author      : pengjiawei
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <Parameter/Parameter.h>
#include <thread>
#include "SeExplorer.h"
using namespace std;
inline static bool operator==(const NS_DataType::Point& one,
                              const NS_DataType::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}
namespace NS_Explorer{
ExplorerApplication::ExplorerApplication()
{
	map_cli = new NS_Service::Client<NS_ServiceType::ServiceMap> ("MAP");

//	resolution_ = srv_map.map.info.resolution;
	resolution_ = 0.1;
//  explore_costmap_ = new NS_CostMap::CostmapWrapper ();
//  explore_costmap_->initialize ();
	loadParameter();

//  explore_costmap_->start ();

	goal_pub = new NS_DataSet::Publisher< NS_DataType::Pose >("GOAL_FROM_APP");

	explore_sub = new NS_DataSet::Subscriber< bool >("IS_EXPLORING",boost::bind(&ExplorerApplication::isExploringCallback, this, _1));
}

ExplorerApplication::~ExplorerApplication(){
	delete goal_pub;
	delete explore_sub;
}

void ExplorerApplication::run(){
	NS_ServiceType::ServiceMap srv_map;
//	map_cli->call(srv_map);
	search_ = frontier_exploration::FrontierSearch(srv_map,
	                                               potential_scale_, gain_scale_,
	                                               size_t(min_frontier_size));
	running = true;
	printf("start to explore,make plan()\n");
	makePlan();
}
void ExplorerApplication::quit(){
	running = false;
	printf("quit ExplorerApplication\n");
}
void ExplorerApplication::makePlan() {

	  sleep(2);
	  printf("sleep two seconds ohhhhhhhhhhhhhhhhhhh,begin to make plan\n");
	  // find frontiers
	  NS_Transform::Stamped<NS_Transform::Pose> pose;
	//  explore_costmap_->getRobotPose(pose);
	  NS_DataType::Point p;
	  p.x = pose.getOrigin().getX();
	  p.y = pose.getOrigin().getY();
	  p.z = pose.getOrigin().getZ();

	  p.x = 0.0;
	  p.y = 0.0;
	  p.z = 0.0;
	  printf("current pose = (%.4f,%.4f,%.4f)\n",p.x,p.y,p.z);
	  auto frontiers = search_.searchFrom(p);
	  printf("found %lu frontiers\n", frontiers.size());
	  for (size_t i = 0; i < frontiers.size(); ++i) {
	    printf("frontier %zd cost: %f\n", i, frontiers[i].cost);
	  }

	  if (frontiers.empty()) {
	//    stop();
	    printf("frontiers is empty!\n");
	    return;
	  }

	  // publish frontiers as visualization markers
	//  if (visualize_) {
	//    visualizeFrontiers(frontiers);
	//  }

	  // find non blacklisted frontier
	  auto frontier =
	      std::find_if_not(frontiers.begin(), frontiers.end(),
	                       [this](const frontier_exploration::Frontier& f) {
	                         return goalOnBlacklist(f.centroid);
	                       });
	  if (frontier == frontiers.end()) {
	//    stop();
	    printf("find non blacklisted frontier is null\n");
	    return;
	  }
	  // todo sort frontiers
	  NS_DataType::Point target_position = frontier->centroid;
	  printf("frontier min_distance = %.4f\n",frontier->min_distance);
	  // time out if we are not making any progress
	  bool same_goal = (prev_goal_ == target_position);
	  prev_goal_ = target_position;
	  if (!same_goal || prev_distance_ > frontier->min_distance) {
	    // we have different goal or we made some progress
	    last_progress_ = NS_NaviCommon::Time::now();
	    prev_distance_ = frontier->min_distance;
	  }
	  // black list if we've made no progress for a long time
	  if (NS_NaviCommon::Time::now() - last_progress_ > progress_timeout_) {
	    frontier_blacklist_.push_back(target_position);
	    printf("----if we made no progress for a long time (progress_timeout)-----Adding current goal to black list and make plan again!\n");
	    makePlan();
	    return;
	  }

	  // we don't need to do anything if we still pursuing the same goal
	  if (same_goal) {
	    printf("same_goal we dont pursue it\n");
	    return;
	  }

	  // send goal to move_base if we have something new to pursue
	  NS_DataType::PoseStamped goal_pose;
	  goal_pose.pose.position = target_position;
	  goal_pose.pose.orientation.w = 1;
	  goal_pose.header.stamp = NS_NaviCommon::Time::now();
	  printf("goal_pose = (%.4f,%.4f)\n",goal_pose.pose.position.x,goal_pose.pose.position.y);

	  NS_DataType::Pose published_pose;
	  published_pose.position.x = goal_pose.pose.position.x;
	  published_pose.position.y = goal_pose.pose.position.y;
	  published_pose.orientation = NS_Transform::createQuaternionMsgFromYaw(0.0);
	  printf("print goal to global planner------------------\n");
	  goal_pub->publish(published_pose);
	  NS_DataType::Point middle_point = frontier->middle;
	  printf("middle_point x = %.4f,y = %.4f\n",middle_point.x,middle_point.y);
	  std::vector<NS_DataType::Point> final_points = frontier->points;
	  FILE* final_points_file;
	  final_points_file = fopen("/tmp/points.log","w");
	  for (unsigned int i = 0;i < final_points.size();++i){
	    fprintf(final_points_file,"%.4f %.4f\n",final_points[i].x,final_points[i].y);
	  }
	  fclose(final_points_file);
	//  move_base_msgs::MoveBaseGoal goal;
	//  goal.target_pose.pose.position = target_position;
	//  goal.target_pose.pose.orientation.w = 1.;
	//  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
	//  goal.target_pose.header.stamp = ros::Time::now();
	//  move_base_client_.sendGoal(
	//      goal, [this, target_position](
	//                const actionlib::SimpleClientGoalState& status,
	//                const move_base_msgs::MoveBaseResultConstPtr& result) {
	//        reachedGoal(status, result, target_position);
	//      });

}

void ExplorerApplication::loadParameter() {

	NS_NaviCommon::Parameter parameter;

	planner_frequency_ = parameter.getParameter ("planner_frequency", 1.0f);

	progress_timeout_ = NS_NaviCommon::Duration(parameter.getParameter ("progress_timeout", 30.0f));


	potential_scale_ = parameter.getParameter ("potential_scale", 0.001f);
	orientation_scale_ = parameter.getParameter ("orientation_scale", 0.0f);

	gain_scale_ = parameter.getParameter ("gain_scale", 1.0f);
	min_frontier_size = parameter.getParameter ("min_frontier_size", 10);

	prev_distance_ = parameter.getParameter("prev_distance",0);
}

bool ExplorerApplication::goalOnBlacklist(
		const NS_DataType::Point& goal) {

	  constexpr static size_t tolerace = 5;


	  // check if a goal is on the blacklist for goals that we're pursuing
	  for (auto& frontier_goal : frontier_blacklist_) {
	    double x_diff = fabs(goal.x - frontier_goal.x);
	    double y_diff = fabs(goal.y - frontier_goal.y);

	    if (x_diff < tolerace * resolution_ &&
	        y_diff < tolerace * resolution_)
	      return true;
	  }
	  return false;

}

void ExplorerApplication::isExploringCallback(bool isExploring){
	printf("is exploring callback = %d\n",isExploring);
	if(isExploring){
		makePlan();
	}
}
}
//int main() {
//	NS_Explorer::ExplorerApplication* explorer = new NS_Explorer::ExplorerApplication();
//	explorer->makePlan();
//	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
//	return 0;
//}


