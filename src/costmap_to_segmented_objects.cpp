#include <costmap_converter/costmap_to_segmented_objects.h>

#include <utility>

namespace costmap_converter
{

CostmapToSegmentedObjects::CostmapToSegmentedObjects():
  CostmapToPolygonsDBSMCCH::CostmapToPolygonsDBSMCCH(),
  dynamic_recfg_so_(nullptr) {
}

CostmapToSegmentedObjects::~CostmapToSegmentedObjects() {
  if (dynamic_recfg_so_ != nullptr) {
    delete dynamic_recfg_so_;
  }
}

void CostmapToSegmentedObjects::initialize(ros::NodeHandle nh) {
    CostmapToPolygonsDBSMCCH::initialize(nh);

    // setup dynamic reconfigure
    // separate NH due to dynamic reconfigure services conflicts;
    // suffix meaning - Segmented Objects Dynamic Reconfigure
    dynamic_recfg_so_ = new dynamic_reconfigure::Server<CostmapToSegmentedObjectsConfig>(ros::NodeHandle(nh, nh.getNamespace() + "SODR"));
    dynamic_reconfigure::Server<CostmapToSegmentedObjectsConfig>::CallbackType cb = boost::bind(&CostmapToSegmentedObjects::reconfigureCB, this, _1, _2);
    dynamic_recfg_so_->setCallback(cb);
}

void CostmapToSegmentedObjects::compute() {
  CostmapToPolygonsDBSMCCH::compute();

  // Algorithm implemented below glues up polygons
  // that are close enough to each other (within segmentation_distance)

  // stores visited pairs of polygons
  std::vector<std::pair<size_t, size_t>> visited_pairs;
  // stores indices of pairs that can be glued together
  std::vector<std::pair<size_t, size_t>> extension_pairs;

  auto polygons = *getPolygons();
  size_t polygons_num_init = polygons.size();

  size_t id_set_a = 0;
  size_t id_set_b = 0;

  // 1st stage
  // this loop finds pairs of polygons whose closest vertices are close enough (below the threshold)
  for (const auto& polygon_a: polygons) {
    // first indices will be equal to 1 (not 0 - to simplify operations if loop is skipped)
    id_set_a++;
    for (const auto& polygon_b: polygons) {
      id_set_b++;

      // check if polygon pair was visited before
      auto it_visited = std::find_if(
        visited_pairs.begin(),
        visited_pairs.end(),
        [&id_set_a, &id_set_b](const std::pair<size_t, size_t>& element) {
          return element.first == id_set_a && element.second == id_set_b;
        }
      );

      // check if pair was already visited
      if (it_visited != visited_pairs.end()) {
        continue;
      }

      visited_pairs.push_back(std::make_pair(id_set_a, id_set_b));

      // check each pair of points
      for (const auto& pt_a: polygon_a.points) {
        for (const auto& pt_b: polygon_b.points) {
          auto dist = std::hypot(pt_b.x - pt_a.x, pt_b.y - pt_a.y);
          if (dist <= parameter_so_.segmentation_distance_) {
            extension_pairs.push_back(std::make_pair(id_set_a, id_set_b));
            continue;
          }
        }
      }
    }
  }

  // make sure that further actions are necessary (i.e. check if some polygons can be merged)
  if (extension_pairs.empty()) {
    return;
  }

  // 2nd stage
  // create polygon groups, select polygons that can be glued together, group ID is the key here
  std::map<size_t, std::vector<size_t>> polygon_groups;

  // `extension_pairs` must not to contain repeated pairs
  for (const auto& polygon_pair: extension_pairs) {
    // check if any polygon ID from the `polygon_pair` was already put into any polygon group
    auto it_group_first_elem = std::find_if(
      polygon_groups.begin(),
      polygon_groups.end(),
      [&polygon_pair](const std::pair<size_t, std::vector<size_t>>& group) {
        return std::find(group.second.begin(), group.second.end(), polygon_pair.first) != group.second.end();
      }
    );
    auto it_group_second_elem = std::find_if(
      polygon_groups.begin(),
      polygon_groups.end(),
      [&polygon_pair](const std::pair<size_t, std::vector<size_t>>& group) {
        return std::find(group.second.begin(), group.second.end(), polygon_pair.second) != group.second.end();
      }
    );

    // check if none of the IDs assigned to any group
    if (it_group_first_elem == polygon_groups.end() && it_group_second_elem == polygon_groups.end()) {
      // initiate group with 2 IDs
      polygon_groups[polygon_groups.size()] = std::vector<size_t>{polygon_pair.first, polygon_pair.second};
      continue;
    }

    // one of the IDs assigned to some group
    bool first_id_found = it_group_first_elem != polygon_groups.end();
    bool second_id_found = it_group_second_elem != polygon_groups.end();

    // add second polygon ID to the group related to first ID and vice versa
    if (first_id_found && !second_id_found) {
      it_group_first_elem->second.push_back(polygon_pair.second);
    } else if (!first_id_found && second_id_found) {
      it_group_second_elem->second.push_back(polygon_pair.first);
    }
  }

  // 3rd stage
  // create groups with all polygons (also single ones)
  for (size_t id = 1; id <= polygons_num_init; id++) {
    // check if group with `id` exists
    auto it_group_with_id = std::find_if(
      polygon_groups.begin(),
      polygon_groups.end(),
      [&id](const std::pair<size_t, std::vector<size_t>>& group) {
        return std::find(group.second.begin(), group.second.end(), id) != group.second.end();
      }
    );

    // group with `id` found
    if (it_group_with_id != polygon_groups.end()) {
      continue;
    }

    // initiate new polygon group that will contain only 1 element
    polygon_groups[polygon_groups.size()] = std::vector<size_t>{id};
  }

  // 4th stage
  // prepare new polygon set based on polygon groups defined previously
  // create new polygon container (for modified data)
  PolygonContainerPtr polygons_segmented(new std::vector<geometry_msgs::Polygon>());
  for (const auto& group: polygon_groups) {
    // temporary polygon
    geometry_msgs::Polygon tp;
    // iterate over IDs of polygons assigned to the group
    for (const auto& id: group.second) {
      // iterate over polygon's vertices
      // compensate index offset introduced in the 1st stage
      for (const auto& vertex: polygons.at(id - 1).points) {
        tp.points.push_back(vertex);
      }
    }
    polygons_segmented->push_back(tp);
  }

  // update existing polygons set with the newly created one
  boost::mutex::scoped_lock lock(mutex_);
  polygons_ = polygons_segmented;
}

void CostmapToSegmentedObjects::updateCostmap2D() {
  // get a copy of our parameters from dynamic reconfigure
  boost::mutex::scoped_lock lock(parameter_mutex_);
  parameter_so_ = parameter_so_buffered_;

  CostmapToPolygonsDBSMCCH::updateCostmap2D();
}

void CostmapToSegmentedObjects::reconfigureCB(CostmapToSegmentedObjectsConfig& config, uint32_t level) {
  boost::mutex::scoped_lock lock(parameter_mutex_);
  parameter_so_buffered_.segmentation_distance_ = config.segmentation_distance;
}

} // end namespace costmap_converter
