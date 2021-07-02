// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <ros/ros.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_ros/pcl_nodelet.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_recognition_utils/pcl_util.h"
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_pcl_ros/tf_listener_singleton.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <nav_msgs/Odometry.h>
#include <aerial_robot_pcl/GroundPlaneDetectionConfig.h>

namespace aerial_robot_pcl
{
    class GroundPlaneDetection: public jsk_topic_tools::DiagnosticNodelet
    {
    public:
        typedef pcl::PointXYZRGB PointT;
        typedef aerial_robot_pcl::GroundPlaneDetectionConfig Config;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;

        GroundPlaneDetection(): DiagnosticNodelet("GroundPlaneDetection") {}

    protected:
        /* ros publisher */
        ros::Publisher pub_inliers_, pub_coefficients_, pub_polygons_;

        /* ros subscriber */
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
        message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;
        boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
        boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
        tf::TransformListener* tf_listener_;

        /* ros param */
        double outlier_threshold_;
        int min_inliners_;
        int max_interations_;
        int min_trial_;
        double esp_angle_;
        bool dynamic_sensor_tf_;
        std::string tf_prefix_;
        std::string sensor_tf_name_;

        virtual void onInit();
        virtual void subscribe();
        virtual void unsubscribe();
        virtual void configCallback (Config &config, uint32_t level);
        virtual void segment(const sensor_msgs::PointCloud2::ConstPtr& msg_points, const nav_msgs::Odometry::ConstPtr& msg_odom);
        virtual void publishResult(const std_msgs::Header& header,
                                   const std::vector<pcl::PointIndices::Ptr>& inliers,
                                   const std::vector<pcl::ModelCoefficients::Ptr>& coefficients,
                                   const std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes);

    };

} //namespace aerial_robot_perception
