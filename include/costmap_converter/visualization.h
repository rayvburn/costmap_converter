#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>

#include <string>

#include <costmap_converter/costmap_converter_interface.h>

namespace costmap_converter {

static void publishAsMarker(const std::string& frame_id, const std::vector<geometry_msgs::PolygonStamped>& polygonStamped, ros::Publisher& marker_pub)
{
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = frame_id;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "Polygons";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  line_list.scale.x = 0.1;
  line_list.color.g = 1.0;
  line_list.color.a = 1.0;

  for (std::size_t i=0; i<polygonStamped.size(); ++i)
  {
    for (int j=0; j< (int)polygonStamped[i].polygon.points.size()-1; ++j)
    {
      geometry_msgs::Point line_start;
      line_start.x = polygonStamped[i].polygon.points[j].x;
      line_start.y = polygonStamped[i].polygon.points[j].y;
      line_list.points.push_back(line_start);
      geometry_msgs::Point line_end;
      line_end.x = polygonStamped[i].polygon.points[j+1].x;
      line_end.y = polygonStamped[i].polygon.points[j+1].y;
      line_list.points.push_back(line_end);
    }
    // close loop for current polygon
    if (!polygonStamped[i].polygon.points.empty() && polygonStamped[i].polygon.points.size() != 2 )
    {
      geometry_msgs::Point line_start;
      line_start.x = polygonStamped[i].polygon.points.back().x;
      line_start.y = polygonStamped[i].polygon.points.back().y;
      line_list.points.push_back(line_start);
      if (line_list.points.size() % 2 != 0)
      {
        geometry_msgs::Point line_end;
        line_end.x = polygonStamped[i].polygon.points.front().x;
        line_end.y = polygonStamped[i].polygon.points.front().y;
        line_list.points.push_back(line_end);
      }
    }


  }
  marker_pub.publish(line_list);
}

static void publishAsMarker(const std::string& frame_id, const costmap_converter::ObstacleArrayMsg& obstacles, ros::Publisher& marker_pub)
{
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = frame_id;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "Polygons";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;

  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  line_list.scale.x = 0.1;
  line_list.color.g = 1.0;
  line_list.color.a = 1.0;

  for (const costmap_converter::ObstacleMsg& obstacle : obstacles.obstacles)
  {
    for (int j=0; j< (int)obstacle.polygon.points.size()-1; ++j)
    {
      geometry_msgs::Point line_start;
      line_start.x = obstacle.polygon.points[j].x;
      line_start.y = obstacle.polygon.points[j].y;
      line_list.points.push_back(line_start);
      geometry_msgs::Point line_end;
      line_end.x = obstacle.polygon.points[j+1].x;
      line_end.y = obstacle.polygon.points[j+1].y;
      line_list.points.push_back(line_end);
    }
    // close loop for current polygon
    if (!obstacle.polygon.points.empty() && obstacle.polygon.points.size() != 2 )
    {
      geometry_msgs::Point line_start;
      line_start.x = obstacle.polygon.points.back().x;
      line_start.y = obstacle.polygon.points.back().y;
      line_list.points.push_back(line_start);
      if (line_list.points.size() % 2 != 0)
      {
        geometry_msgs::Point line_end;
        line_end.x = obstacle.polygon.points.front().x;
        line_end.y = obstacle.polygon.points.front().y;
        line_list.points.push_back(line_end);
      }
    }
  }
  marker_pub.publish(line_list);
}

} // end namespace costmap_converter
