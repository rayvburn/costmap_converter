#include <costmap_converter/costmap_to_segmented_objects.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToSegmentedPolygonsDBSMCCH, costmap_converter::BaseCostmapToPolygons);
PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToSegmentedPolygonsDBSConcaveHull, costmap_converter::BaseCostmapToPolygons);
PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToSegmentedLinesDBSRANSAC, costmap_converter::BaseCostmapToPolygons);
PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToSegmentedLinesDBSMCCH, costmap_converter::BaseCostmapToPolygons);

namespace costmap_converter
{
// outside class initialization of the static member
size_t PolygonIdentified::polygon_num_ = 0;
} // namespace costmap_converter
