#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include <DataSet/DataType/Point.h>
#include <Service/ServiceType/ServiceMap.h>
#include <Service/Client.h>
namespace frontier_exploration
{
/**
 * @brief Represents a frontier
 *
 */
struct Frontier {
//  std::uint32_t size;
  unsigned int size;
  double min_distance;
  double cost;
  NS_DataType::Point initial;
  NS_DataType::Point centroid;
  NS_DataType::Point middle;
  std::vector<NS_DataType::Point> points;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch()
  {
  }

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  FrontierSearch(NS_ServiceType::ServiceMap& costmap_wrapper, double potential_scale,
                 double gain_scale, size_t min_frontier_size);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(NS_DataType::Point position);

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                            std::vector<bool>& frontier_flag);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier);

private:
  NS_ServiceType::ServiceMap service_map_;
  unsigned char* map_;
  //parameter of costmap
    double resolution_;
    double origin_x_;
    double origin_y_;
    unsigned int size_x_;
    unsigned int size_y_;
  double potential_scale_, gain_scale_;
  NS_Service::Client< vector<unsigned char> >* char_map_cli;
  size_t min_frontier_size_;
};
}
#endif
