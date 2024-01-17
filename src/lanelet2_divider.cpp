//
// Created by ataparlar on 04.01.2024.
//

#include "lanelet2_divider/lanelet2_divider.hpp"

#include "GeographicLib/UTMUPS.hpp"
#include "ogrsf_frmts.h"

#include <ogr_feature.h>

Lanelet2Divider::Lanelet2Divider() : Node("lanelet2_divider")
{
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("mgrs_grid_path", 10);

  GDALAllRegister();

  GDALDataset * poDS;
  poDS = (GDALDataset *)GDALOpenEx(
    "/home/ataparlar/data/gis_data/mgrs_grids/UTM_Datum_WGS84_35T_polygons.gpkg", GDAL_OF_VECTOR,
    NULL, NULL, NULL);
  if (poDS == NULL) {
    printf("Open failed.\n");
    exit(1);
  }

  OGRLayer * gridLayer;
  gridLayer = poDS->GetLayerByName("35T_100km");

  OGRFeature * ist_grid_feature = gridLayer->GetFeature(42);
  std::cout << ist_grid_feature->GetFieldAsString("MGRS_100") << std::endl;  // 35TPF

  OGRGeometry * ist_grid = ist_grid_feature->GetGeometryRef();
  std::cout << ist_grid->getGeometryName() << std::endl;

  OGRMultiPolygon * ist_polygons = ist_grid->toMultiPolygon();
  std::cout << ist_polygons->getGeometryName() << std::endl;

  nav_msgs::msg::Path path_;
  path_.header.frame_id = "map";
  path_.header.stamp = this->get_clock()->now();

  for (OGRPolygon * ist_poly : ist_polygons) {
    std::cout << ist_poly->getGeometryName() << std::endl;
    std::cout << "ist_poly:" << std::endl;
    std::cout << ist_poly->get_Area() << std::endl;

    for (OGRLinearRing * linearring : ist_poly) {
      std::cout << "\t" << linearring->getGeometryName() << std::endl;

      for (const OGRPoint & point : linearring) {
        double origin_x, origin_y, x, y;
        if (!origin_init_) {
          int zone;
          bool northp;
          double gamma, k;
          GeographicLib::UTMUPS::Forward(
            point.getY(), point.getX(), zone, northp, origin_x, origin_y, gamma, k);
          origin_init_ = true;
        } else {
          int zone;
          bool northp;
          double gamma, k;
          GeographicLib::UTMUPS::Forward(point.getY(), point.getX(), zone, northp, x, y, gamma, k);
        }

        double local_x = x - origin_x;
        double local_y = y - origin_y;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = local_x;
        pose.pose.position.y = local_y;
        pose.pose.position.z = 0;

        path_.poses.push_back(pose);

        std::cout << "\t\t" << std::setprecision(12) << point.getGeometryName() << std::endl;
        std::cout << "\t\tX: " << std::setprecision(12) << point.getX() << std::endl;
        std::cout << "\t\tY: " << std::setprecision(12) << point.getY() << std::endl;
        std::cout << "\t\tZ: " << std::setprecision(12) << point.getZ() << std::endl;
        std::cout << "\t\tlocal_x: " << std::setprecision(12) << local_x << std::endl;
        std::cout << "\t\tlocal_y: " << std::setprecision(12) << local_y << std::endl;
        std::cout << "\t\tpath_.poses.size: " << path_.poses.size() << std::endl;

//        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }
  path_pub_->publish(path_);
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lanelet2Divider>());
  rclcpp::shutdown();
  return 0;
}