/*
 * Copyright (c) 2023 Haruki Kozuka <kozuka@jsk.imi.i.u-tokyo.ac.jp>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 */


#ifndef LASER_SCAN_TREE_RANGE_DIFF_FILTER_H
#define LASER_SCAN_TREE_RANGE_DIFF_FILTER_H

#include <set>

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include <angles/angles.h>

using namespace std;

namespace laser_filters{

/** @b LaserClusteringFilter is a simple filter that filters shadow points in a laser scan line
 */

class LaserRangeDiffFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  double distance_threshold_;          // Filter distance threshold

  ////////////////////////////////////////////////////////////////////////////////
  LaserRangeDiffFilter()
  {


  }

  /**@b Configure the filter from XML */
  bool configure()
  {
    if (!getParam("distance_threshold", distance_threshold_)) {
        ROS_ERROR("LaserRangeDiffFilter: Parameter 'distance_threshold' not set");
        return false;
    }
    ROS_INFO("LaserRangeDiffFilter: Configured with distance_threshold = %f", distance_threshold_);
    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////
  virtual ~LaserRangeDiffFilter() { }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief
   * \param scan_in the input LaserScan message
   * \param scan_out the output LaserScan message
   */
  bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
  {
    scan_out = scan_in; //copy across all data first

    for (size_t i = 1; i < scan_in.ranges.size(); ++i) {
        float current_point = scan_in.ranges[i];
        float previous_point = scan_in.ranges[i - 1];

        if (std::abs(current_point - previous_point) > distance_threshold_) {
            scan_out.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
    }
    return true;
  }


} ;
}

#endif //LASER_CUSTERING_FILTER_H
