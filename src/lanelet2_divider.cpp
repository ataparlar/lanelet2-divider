//
// Created by ataparlar on 04.01.2024.
//

#include "lanelet2_divider/lanelet2_divider.hpp"

#include "GeographicLib/UTMUPS.hpp"
#include "ogrsf_frmts.h"

#include <ogr_feature.h>

Lanelet2Divider::Lanelet2Divider() : Node("lanelet2_divider")
{
  path_pub_100km_ = this->create_publisher<nav_msgs::msg::Path>("mgrs_grid_path_100km", 10);
  path_pub_10km_ = this->create_publisher<nav_msgs::msg::Path>("mgrs_grid_path_10km", 10);
  marker_pub_points_ =
    this->create_publisher<visualization_msgs::msg::Marker>("mgrs_grid_marker_points", 10);
  marker_pub_polygons_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("mgrs_grid_marker_polygons", 10);

  visualization_msgs::msg::MarkerArray polygons;
  visualization_msgs::msg::Marker points, line_list;
  points.header.frame_id = line_list.header.frame_id = "map";
  points.header.stamp = line_list.header.stamp = this->get_clock()->now();
  points.ns = "points";
  points.action = line_list.action = visualization_msgs::msg::Marker::ADD;
  points.id = 0;
  points.type = visualization_msgs::msg::Marker::POINTS;
  line_list.type = visualization_msgs::msg::Marker::LINE_LIST;
  points.scale.x = 40;
  points.scale.y = 40;
  line_list.scale.x = 100;
  points.color.b = 1.0f;
  points.color.a = 1.0;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  // 41.0873,28.8290
  int zone;
  bool northp;
  double origin_x, origin_y, gamma, k;
  GeographicLib::UTMUPS::Forward(41.0873, 28.8290, zone, northp, origin_x, origin_y, gamma, k);

  GDALAllRegister();

  GDALDataset * poDS;
  poDS = (GDALDataset *)GDALOpenEx(
    "/home/ataparlar/data/gis_data/mgrs_grids/UTM_Datum_WGS84_35T_polygons.gpkg", GDAL_OF_VECTOR,
    NULL, NULL, NULL);
  if (poDS == NULL) {
    printf("Open failed.\n");
    exit(1);
  }

  // 100 km Grid
  OGRLayer * gridLayer = poDS->GetLayerByName("35T_100km");
  OGRFeature * ist_grid_feature = gridLayer->GetFeature(42);
  OGRGeometry * ist_grid = ist_grid_feature->GetGeometryRef();
  OGRMultiPolygon * ist_polygons = ist_grid->toMultiPolygon();

  nav_msgs::msg::Path path_100km_;
  path_100km_.header.frame_id = "map";
  path_100km_.header.stamp = this->get_clock()->now();

  for (OGRPolygon * ist_poly : ist_polygons) {
    for (OGRLinearRing * linearring : ist_poly) {
      for (const OGRPoint & point : linearring) {
        double x, y;
        GeographicLib::UTMUPS::Forward(point.getY(), point.getX(), zone, northp, x, y, gamma, k);

        double local_x = x - origin_x;
        double local_y = y - origin_y;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = local_x;
        pose.pose.position.y = local_y;
        pose.pose.position.z = 0;

        path_100km_.poses.push_back(pose);
      }
    }
    path_pub_100km_->publish(path_100km_);
  }

  // 10 km Grids
  OGRLayer * gridLayer_ = poDS->GetLayerByName("35T_10km");

  std::cout << "Count before filter:  " << gridLayer_->GetFeatureCount() << std::endl;
  gridLayer_->SetAttributeFilter("MGRS_10 LIKE '35TPF%'");
  std::cout << "Count after filter:  " << gridLayer_->GetFeatureCount() << std::endl;

  nav_msgs::msg::Path path_10km_;
  path_10km_.header.frame_id = "map";
  path_10km_.header.stamp = this->get_clock()->now();

  for (auto & feat : gridLayer_) {
    line_list.ns = feat->GetFieldAsString("MGRS_10");
    int marker_id = 1;
    OGRGeometry * geometry = feat->GetGeometryRef();
    OGRMultiPolygon * multiPolygon = geometry->toMultiPolygon();
    for (OGRPolygon * polygon : multiPolygon) {
      for (OGRLinearRing * linearring : polygon) {
        for (const OGRPoint & point : linearring) {
          double x, y;
          GeographicLib::UTMUPS::Forward(point.getY(), point.getX(), zone, northp, x, y, gamma, k);
          double local_x = x - origin_x;
          double local_y = y - origin_y;

          geometry_msgs::msg::Point p;
          p.x = local_x;
          p.y = local_y;
          p.z = 0;
          points.points.push_back(p);
          line_list.id = marker_id;
          marker_id++;
          line_list.points.push_back(p);

          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = "map";
          pose.header.stamp = this->get_clock()->now();
          pose.pose.position.x = local_x;
          pose.pose.position.y = local_y;
          pose.pose.position.z = 0;

          path_10km_.poses.push_back(pose);
        }
      }
    }
    line_list.points.pop_back();
    polygons.markers.push_back(line_list);
    line_list.points.clear();

    marker_pub_points_->publish(points);
    marker_pub_polygons_->publish(polygons);
    path_pub_10km_->publish(path_10km_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lanelet2Divider>());
  rclcpp::shutdown();
  return 0;
}