#include "algorithmAPI.h"
#include<stdio.h>
bool checkWindowValid(const sensor_msgs::msg::LaserScan &scan, size_t idx, size_t window, double max_range_difference)
{
  const float &range = scan.ranges[idx];
  if (range != range)
  {
    return false;
  }

  size_t i = idx + 1;
  size_t i_max = std::min(idx + window, scan.ranges.size());
  while (i < i_max)
  {
    const float &neighbor_range = scan.ranges[i];
    if (neighbor_range != neighbor_range || fabs(neighbor_range - range) > max_range_difference)
    {
      return false;
    }
    ++i;
  }

  return true;
}

bool checkWindowValid2(const sensor_msgs::msg::LaserScan &scan, size_t idx, size_t window, double max_distance)
{
  int num_neighbors = 0;
  const float &r1 = scan.ranges[idx]; // 当前点云的距离数据
  float r2 = 0.;                      // 范围内点云的数据

  // Look around the current point until either the window is exceeded
  // or the number of neighbors was found.
  for (int y = -(int)window; y < (int)window + 1 && num_neighbors < (int)window; y++)
  {
    int j = idx + y;
    r2 = scan.ranges[j];

    if (j < 0 || j >= static_cast<int>(scan.ranges.size()) || idx == j || std::isnan(r2)) // skip the point
    {                                                                                     // Out of scan bounds or itself or infinity
      continue;
    }
    // delete the re in the result
    const float d = sqrt(
        pow(r1, 2) + pow(r2, 2) -
        (2 * r1 * r2 * cosf(y * scan.angle_increment)));

    if (d <= max_distance)
    {
      num_neighbors++;
    }
  }

  // consider the window to be the number of neighbors we need
  if (num_neighbors < window)
  // if(num_neighbors == 0)//only the window is one
  {
    return false; // invalid
  }
  else
  {
    return true; // effective
  }
}

bool filter(const sensor_msgs::msg::LaserScan &input_scan, sensor_msgs::msg::LaserScan &output_scan, int filter_type, double max_range, double min_range, double max_range_difference, int filter_window)
{
  output_scan= input_scan;

  std::vector<bool> valid_ranges;

  /*Check if range size is big enough to use7 the filter window */
  if (output_scan.ranges.size() <= filter_window + 1)
  {
    printf("Scan ranges size is too small: size = %ld", output_scan.ranges.size());
    return false;
  }

  size_t i = 0;
  size_t i_max = input_scan.ranges.size();
  valid_ranges.clear();
  while (i < i_max)
  {
    bool out_of_range = ((output_scan.ranges[i] > max_range) || (output_scan.ranges[i] < min_range));
    // ROS_INFO("%lf", min_range);
    valid_ranges.push_back(out_of_range);
    ++i;
  }

  i = 0;
  i_max = input_scan.ranges.size() - filter_window + 1;
  while (i < i_max)
  {
    bool window_valid;
    if (filter_type == 0)
      window_valid = checkWindowValid(output_scan, i, filter_window, max_range_difference);
    else
      window_valid = checkWindowValid2(output_scan, i, filter_window, max_range_difference);

    if (window_valid)
    {
      size_t j = i, j_max = i + filter_window;
      do
      {
        valid_ranges[j++] = true;
      } while (j < j_max);
    }
    ++i;
  }

  i = 0;
  i_max = valid_ranges.size();
  int errnum=0;
  while (i < i_max)
  {
    if (!valid_ranges[i])
    {
      output_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      errnum++;
    }
    ++i;
  }
  printf("%ld not valid num:%d\n",i,errnum);
  return true;
}
