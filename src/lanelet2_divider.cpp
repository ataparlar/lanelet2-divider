//
// Created by ataparlar on 04.01.2024.
//

#include "lanelet2_divider/lanelet2_divider.hpp"

#include "ogrsf_frmts.h"

#include <ogr_feature.h>

Lanelet2Divider::Lanelet2Divider() : Node("lanelet2_divider")
{
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


  for (OGRPolygon * ist_poly: ist_polygons) {
    std::cout << ist_poly->getGeometryName() << std::endl;
    std::cout << "ist_poly:" << std::endl;
    std::cout << ist_poly->get_Area() << std::endl;

    for (OGRLinearRing * linearring : ist_poly) {
      std::cout << "\t" << linearring->getGeometryName() << std::endl;
      for (const OGRPoint & point : linearring) {
        std::cout << "\t\t" << point.getGeometryName() << std::endl;
        std::cout << "\t\tX: " << point.getX() << std::endl;
        std::cout << "\t\tY: " << point.getY() << std::endl;
        std::cout << "\t\tZ: " << point.getZ() << std::endl;
      }
    }

  }
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lanelet2Divider>());
  rclcpp::shutdown();
  return 0;
}