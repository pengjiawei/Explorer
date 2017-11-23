/**
 * attention : what to do with this rely on costMap
 */
#ifndef SEEXPLORER_H_
#define SEEXPLORER_H_

#include <memory>

#include <string>
#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <Application/Application.h>
#include <DataSet/DataType/PoseStamped.h>
#include <DataSet/DataType/Pose.h>
#include <DataSet/DataType/Point.h>
#include <Time/Time.h>
#include <Time/Duration.h>
#include <DataSet/Publisher.h>
#include <DataSet/Subscriber.h>
#include <Service/ServiceType/ServiceMap.h>
#include <algorithm>
#include <Transform/DataTypes.h>
#include "bfs_explorer/frontier_search.h"

namespace NS_Explorer {
class ExplorerApplication : public Application{
public:
	ExplorerApplication();
	virtual
	~ExplorerApplication();
	/**
	 * @brief  Make a global plan
	 */
	void makePlan();

	void run();

	void quit();

	bool isRunning(){
		return running;
	}
private:
	void loadParameter();
	/**
	   * @brief  Publish a frontiers as markers
	   */
	  void visualizeFrontiers(
	      const std::vector<frontier_exploration::Frontier>& frontiers);

	//  void reachedGoal(const actionlib::SimpleClientGoalState& status,
	//                   const move_base_msgs::MoveBaseResultConstPtr& result,
	//                   const geometry_msgs::Point& frontier_goal);

	  bool goalOnBlacklist(const NS_DataType::Point& goal);

	  void isExploringCallback(bool isExploring);
	  frontier_exploration::FrontierSearch search_;
	//  ros::Timer exploring_timer_;
	//  ros::Timer oneshot_;

	  void listenningThread();
	  std::vector<NS_DataType::Point> frontier_blacklist_;
	  NS_DataType::Point prev_goal_;
	  double prev_distance_;
	  NS_NaviCommon::Time last_progress_;
	  size_t last_markers_count_;

	  // parameters
	  double planner_frequency_;
	  double potential_scale_, orientation_scale_, gain_scale_;
	  NS_NaviCommon::Duration progress_timeout_;
	  int min_frontier_size;


	  double resolution_;

	    NS_ServiceType::ServiceMap srv_map_;


	  NS_DataSet::Publisher< NS_DataType::PoseStamped >* goal_pub;

	  NS_Service::Client< NS_ServiceType::ServiceMap >* map_cli;

	  NS_DataSet::Subscriber< bool >* explore_sub;

	  NS_Service::Client< NS_DataType::PoseStamped >* current_pose_cli;


	  bool running;
};
}
#endif
