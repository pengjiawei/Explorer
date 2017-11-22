#include "frontier_search.h"





#include <DataSet/DataType/Point.h>

#include "costmap_tools.h"


using namespace NS_CostMap;

namespace frontier_exploration
{
//using costmap_2d::LETHAL_OBSTACLE;
//using costmap_2d::NO_INFORMATION;
//using costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(NS_ServiceType::ServiceMap& service_map,
                               double potential_scale, double gain_scale,
                               size_t min_frontier_size)
  : service_map_(service_map)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
{

	char_map_cli = new NS_Service::Client< vector<unsigned char> > ("GLOBAL_CHARMAP");
	vector<unsigned char> char_map;
	char_map_cli->call(char_map);
	printf("char_map size = %d\n",char_map.size());
//	size_x_ = service_map_.map.info.width;
//	size_y_ = service_map_.map.info.height;
//	origin_x_ = service_map.map.info.origin.position.x;
//	origin_y_ = service_map.map.info.origin.position.y;
//	resolution_ = service_map.map.info.resolution;
	size_x_ = 480;
	size_y_ = 480;
	origin_x_ = -25.0;
	origin_y_ = -25.0;
	resolution_ = 0.1;
	printf("begin to initialize the costmap and costmap_tools in frontier_search size_x = %d, size_y = %d, origin_x = %.4f,origin_y = %.4f,resolution = %.4f\n",size_x_,size_y_,origin_x_,origin_y_,resolution_);
	map_ = new unsigned char[size_x_ * size_y_];
	unsigned int index = 0;
	for(unsigned int j = 0;j < size_y_; ++j){
		for(unsigned int i = 0; i <size_x_; ++i){
//			map_[index] = service_map.map.data[index];
			map_[index] = char_map[index];
			++index;
		}
	}

	frontier_exploration::initial(resolution_,origin_x_,origin_y_,size_x_,size_y_);
}

std::vector<Frontier> FrontierSearch::searchFrom(NS_DataType::Point position)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!worldToMap(position.x, position.y, mx, my)) {
    printf("Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
//  std::lock_guard<costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getLayeredCostmap()->getCostmap()->getMutex()));
//  boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_->getLayeredCostmap()->getCostmap()->getMutex()));
  printf("------------remove the lock todo assure sync---------\n");

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, map_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    printf("Could not find nearby clear cell to start search\n");
  }
  visited_flag[bfs.front()] = true;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
//      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
      if (map_[nbr] < INSCRIBED_INFLATED_OBSTACLE && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        printf("found new frontier\n");
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        // TODO consider map resolution here
        printf("found new frontier size = %d\n",new_frontier.size);
        if (new_frontier.size >= min_frontier_size_) {
          frontier_list.push_back(new_frontier);
        }
      }
    }
  }

  printf("print visited flag to file when I found the frontiers\n");
  FILE* visit_file;
  visit_file = fopen("/tmp/visited.log","w");
  for(unsigned int i = 0; i < visited_flag.size();++i){
    if(visited_flag[i] == true){
      unsigned int mx,my;
      indexToCells(i,mx,my);
      fprintf(visit_file,"%d %d\n",mx,my);
    }
  }
  fclose(visit_file);
  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  indexToCells(initial_cell, ix, iy);
  mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  indexToCells(reference, rx, ry);
  mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        indexToCells(nbr, mx, my);
        mapToWorld(mx, my, wx, wy);

        NS_DataType::Point point;
        point.x = wx;
        point.y = wy;

        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx)) {
//    if (map_[nbr] == FREE_SPACE) {
    if (map_[nbr] < INSCRIBED_INFLATED_OBSTACLE) {
      std::printf("index = %d is NO_INFORMATION,nbr = %d is FREE_SPACE values = %d\n",idx,nbr,map_[nbr]);
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  return (potential_scale_ * frontier.min_distance *
          resolution_) -
         (gain_scale_ * frontier.size * resolution_);
}
}
