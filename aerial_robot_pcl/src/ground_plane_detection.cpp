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


#include "aerial_robot_pcl/ground_plane_detection.h"


namespace aerial_robot_pcl
{
    void GroundPlaneDetection::onInit() {
        ConnectionBasedNodelet::onInit();

        pnh_->param("dynamic_sensor_tf_", dynamic_sensor_tf_, false);
        pnh_->param("tf_prefix", tf_prefix_, std::string(""));
        pnh_->param("sensor_tf_name", sensor_tf_name_, std::string(""));
        tf_listener_ = jsk_pcl_ros::TfListenerSingleton::getInstance();

        if (!dynamic_sensor_tf_) {
            // get tf of camera
        }

        onInitPostProcess();
    }

    void GroundPlaneDetection::subscribe() {
        sub_input_.subscribe(*pnh_, "input", 1);
        sub_odom_.subscribe(*pnh_, "odom", 1);
        sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
        sync_->connectInput(sub_input_, sub_odom_);
        sync_->registerCallback(boost::bind(&GroundPlaneDetection::segment, this, _1, _2));
    }

    void GroundPlaneDetection::unsubscribe() {
        sub_input_.unsubscribe();
        sub_odom_.unsubscribe();
    }

    void GroundPlaneDetection::segment(const sensor_msgs::PointCloud2::ConstPtr& msg_points, const nav_msgs::Odometry::ConstPtr& msg_odom)
    {
        pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg_points, *input);
        std::vector<pcl::PointIndices::Ptr> inliers;
        std::vector<pcl::ModelCoefficients::Ptr> coefficients;
        std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes;

        if (dynamic_sensor_tf_) {
            // get tf of camera
        }

        //segmentation

        publishResult(msg_points->header, inliers, coefficients, convexes);
    }

    void GroundPlaneDetection::publishResult(const std_msgs::Header& header,
                                             const std::vector<pcl::PointIndices::Ptr>& inliers,
                                             const std::vector<pcl::ModelCoefficients::Ptr>& coefficients,
                                             const std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes)
    {
        jsk_recognition_msgs::ClusterPointIndices ros_indices_output;
        jsk_recognition_msgs::ModelCoefficientsArray ros_coefficients_output;
        jsk_recognition_msgs::PolygonArray ros_polygon_output;
        ros_indices_output.header = header;
        ros_coefficients_output.header = header;
        ros_polygon_output.header = header;
        ros_indices_output.cluster_indices
            = pcl_conversions::convertToROSPointIndices(inliers, header);
        ros_coefficients_output.coefficients
            = pcl_conversions::convertToROSModelCoefficients(coefficients, header);
        pub_inliers_.publish(ros_indices_output);
        pub_coefficients_.publish(ros_coefficients_output);
        for (size_t i = 0; i < convexes.size(); i++) {
            geometry_msgs::PolygonStamped polygon;
            polygon.header = header;
            polygon.polygon = convexes[i]->toROSMsg();
            ros_polygon_output.polygons.push_back(polygon);
        }
        pub_polygons_.publish(ros_polygon_output);
    }

} //namespace aerial_robot_pcl

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aerial_robot_pcl::GroundPlaneDetection, nodelet::Nodelet);

