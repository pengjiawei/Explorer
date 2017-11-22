#ifndef COSTMAP_TOOLS_H_
#define COSTMAP_TOOLS_H_


#include <DataSet/DataType/PointStamped.h>



namespace frontier_exploration
{


double resolution_;
double origin_x_;
double origin_y_;
unsigned int size_x_;
unsigned int size_y_;

void initial(double resolution,double origin_x,double origin_y,unsigned int size_x,unsigned int size_y){
	resolution_ =resolution;
	origin_x_ = origin_x;
	origin_y_ = origin_y;
	size_x_ = size_x;
	size_y_ = size_y;
}

/**
 * @brief Determine 4-connected neighbourhood of an input cell, checking for map
 * edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood4(unsigned int idx) {
  // get 4-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out;

//  unsigned int size_x_ = costmap.getCostmap()->getSizeInCellsX(),
//               size_y_ = costmap.getCostmap()->getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    printf("Evaluating nhood for offmap point\n");
    return out;
  }

  if (idx % size_x_ > 0) {
    out.push_back(idx - 1);
  }
  if (idx % size_x_ < size_x_ - 1) {
    out.push_back(idx + 1);
  }
  if (idx >= size_x_) {
    out.push_back(idx - size_x_);
  }
  if (idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + size_x_);
  }
  return out;
}

/**
 * @brief Determine 8-connected neighbourhood of an input cell, checking for map
 * edges
 * @param idx input cell index
 * @param costmap Reference to map data
 * @return neighbour cell indexes
 */
std::vector<unsigned int> nhood8(unsigned int idx)
{
  // get 8-connected neighbourhood indexes, check for edge of map
  std::vector<unsigned int> out = nhood4(idx);

//  unsigned int size_x_ = costmap.getCostmap()->getSizeInCellsX(),
//               size_y_ = costmap.getCostmap()->getSizeInCellsY();

  if (idx > size_x_ * size_y_ - 1) {
    return out;
  }

  if (idx % size_x_ > 0 && idx >= size_x_) {
    out.push_back(idx - 1 - size_x_);
  }
  if (idx % size_x_ > 0 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx - 1 + size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx >= size_x_) {
    out.push_back(idx + 1 - size_x_);
  }
  if (idx % size_x_ < size_x_ - 1 && idx < size_x_ * (size_y_ - 1)) {
    out.push_back(idx + 1 + size_x_);
  }

  return out;
}

/**
 * @brief Find nearest cell of a specified value
 * @param result Index of located cell
 * @param start Index initial cell to search from
 * @param val Specified value to search for
 * @param costmap Reference to map data
 * @return True if a cell with the requested value was found
 */
bool nearestCell(unsigned int& result, unsigned int start, unsigned char val,
                  const unsigned char* map)
{
//  const unsigned char* map = costmap.getCostmap()->getCharMap();
//  unsigned int size_x_= costmap.getCostmap()->getSizeInCellsX(),
//               size_y_= costmap.getCostmap()->getSizeInCellsY();

  if (start >= size_x_* size_y_) {
    return false;
  }

  // initialize breadth first search
  std::queue<unsigned int> bfs;
  std::vector<bool> visited_flag(size_x_* size_y_, false);

  // push initial cell
  bfs.push(start);
  visited_flag[start] = true;

  // search for neighbouring cell matching value
  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // return if cell of correct value is found
    printf("nearest cell map[%d] = %d\n",idx,map[idx]);
    if (map[idx] == val) {
      result = idx;
      return true;
    }

    // iterate over all adjacent unvisited cells
    for (unsigned nbr : nhood8(idx)) {
      if (!visited_flag[nbr]) {
        bfs.push(nbr);
        visited_flag[nbr] = true;
      }
    }
  }

  return false;
}



bool
worldToMap (double wx, double wy, unsigned int& mx,
                       unsigned int& my)
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (int) ((wx - origin_x_) / resolution_);
  my = (int) ((wy - origin_y_) / resolution_);

  if (mx < size_x_ && my < size_y_)
    return true;

  return false;
}

void
mapToWorld (unsigned int mx, unsigned int my, double& wx,
                       double& wy)
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

inline unsigned int
getIndex (unsigned int mx, unsigned int my)
{
  return my * size_x_ + mx;
}

inline void
indexToCells (unsigned int index, unsigned int& mx, unsigned int& my)
{
  my = index / size_x_;
  mx = index - (my * size_x_);
}



}
#endif
