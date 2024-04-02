#include "../include/algorithmAPI.h"
#include<math.h>
#include<limits>

bool checkWindowValid2(std::vector<DataPoint> scan, size_t idx, size_t window, double max_distance,double angle_increment)
{
 
  int num_neighbors = 0;
  const float r1 = scan.at(idx).distance; // 当前点云的距离数据
  float r2 = 0.;                      // 范围内点云的数据
  // Look around the current point until either the window is exceeded
  // or the number of neighbors was found.
  for (int y = -(int)window; y < (int)window + 1 && num_neighbors < (int)window; y++)
  {
    int j = idx + y;
    if (j < 0 || j >= static_cast<int>(scan.size())||idx == j)
    {
      continue;
    }
    r2 = scan.at(j).distance; 
    if(r2==0)
      continue;
    // delete the re in the result
    const float d = sqrt(pow(r1, 2) + pow(r2, 2) -(2 * r1 * r2 * cosf(y * angle_increment)));
    if (d <= max_distance)
    {
      num_neighbors++;
    }
  }
  //printf("%s %d %d %d\n", __FUNCTION__, __LINE__,num_neighbors,window);
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

bool filter(std::vector<DataPoint>  &output_scan, int filter_type, double max_range, double min_range, double max_range_difference, int filter_window,double angle_increment)
{
  std::vector<bool> valid_ranges;
  /*Check if range size is big enough to use7 the filter window */
  if (output_scan.size() <= filter_window + 1)
  {
    printf("Scan ranges size is too small: size = %ld", output_scan.size());
    return false;
  }
  size_t i = 0;
  size_t i_max = output_scan.size();
  valid_ranges.clear();
  while (i < i_max)
  {
    bool out_of_range = ((output_scan.at(i).distance > max_range) || (output_scan.at(i).distance < min_range));
    // ROS_INFO("%lf", min_range);
    valid_ranges.push_back(out_of_range);
    ++i;
  }
  i = 0;
  i_max = output_scan.size() - filter_window + 1;
  while (i < i_max)
  {
    bool window_valid = checkWindowValid2(output_scan, i, filter_window, max_range_difference,angle_increment);

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
      output_scan[i].distance = std::numeric_limits<uint16_t>::infinity();
      errnum++;
    }
    ++i;
  }
  //printf("%ld not valid num:%d\n",i,errnum);
  return true;
}
bool checkZeroDistance(std::vector<DataPoint>data,float error_scale)
{
  bool result=true;
  int lengthZeroNum = 0;
  for(int i=0;i<data.size();i++)
  {
    if(data.at(i).distance==0)
      lengthZeroNum++;
  }
  if(data.size()*error_scale<lengthZeroNum)
    return  false;

  return true;
}

// CRC32
unsigned int stm32crc(unsigned int *ptr, unsigned int len)
{
	unsigned int xbit, data;
	unsigned int crc32 = 0xFFFFFFFF;
	const unsigned int polynomial = 0x04c11db7;

	for (unsigned int i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (unsigned int bits = 0; bits < 32; bits++)
		{
			if (crc32 & 0x80000000)
			{
				crc32 <<= 1;
				crc32 ^= polynomial;
			}
			else
				crc32 <<= 1;

			if (data & xbit)
				crc32 ^= polynomial;

			xbit >>= 1;
		}
	}
	return crc32;
}


void resample(RawData *dat, int NN)
{
	int *index = new int[NN];
	double *errs = new double[NN];

	for (int i = 0; i < NN; i++)
		index[i] = -1;

	for (int i = 0; i < dat->N; i++)
	{
		if (dat->points[i].distance == 0)
			continue;

		int idx = round(double(i * NN) / dat->N);
		if (idx < NN)
		{
			double err = fabs(double(dat->span * i) / dat->N - double(dat->span * idx) / NN);
			if (index[idx] == -1 || err < errs[idx])
			{
				index[idx] = i;
				errs[idx] = err;
			}
		}
	}

	for (int i = 1; i < NN; i++)
	{
		if (index[i] != -1)
		{
			dat->points[i] = dat->points[index[i]];
		}
		else
		{
			dat->points[i].distance = 0;
			dat->points[i].confidence = 0;
		}
		dat->points[i].degree = dat->angle / 10.0 + (dat->span * i) / (10.0 * NN);
	}

	dat->N = NN;

	delete index;
	delete errs;
}