#pragma once

// include all classes that will be extended by the 'segmented objects' plugin (all must share base interface)
#include <costmap_converter/costmap_to_polygons.h>
#include <costmap_converter/costmap_to_polygons_concave.h>
#include <costmap_converter/costmap_to_lines_ransac.h>
#include <costmap_converter/costmap_to_lines_convex_hull.h>

// dynamic reconfigure
#include <costmap_converter/CostmapToSegmentedObjectsConfig.h>
#include <dynamic_reconfigure/server.h>

#include <utility>

namespace costmap_converter
{
/// Helper class that stores copy of a polygon with its ID
class PolygonIdentified
{
  public:
    PolygonIdentified(const geometry_msgs::Polygon& polygon):
      polygon_(polygon),
      id_(polygon_num_++)
    {
    }

    geometry_msgs::Polygon getPolygon() const
    {
      return polygon_;
    }

    size_t getID() const
    {
      return id_;
    }

    static size_t getPolygonsNum()
    {
      return polygon_num_;
    }

    /// Needs to be called at the start of each processing
    static void resetPolygonsNum()
    {
      polygon_num_ = 0;
    }

  protected:
    const geometry_msgs::Polygon polygon_;
    size_t id_;

  private:
    static size_t polygon_num_;
};

/**
 * @class CostmapToSegmentedObjects
 *
 * This class inherits from @tparam T and combines polygons produced by the base class.
 * Objects are glued together as long as a distance between any of theirs vertices is shorter than the distance threshold.
 *
 * @tparam T name of the costmap_converter class (static costmap converters)
 */
template <class T>
class CostmapToSegmentedObjects : public T
{

  public:
    /**
     * @struct Parameters
     * @brief Defines the parameters of the algorithm
     */
    struct Parameters
    {
      Parameters() : segmentation_distance_(0.4) {}
      double segmentation_distance_; //!< Maximum distance to neighbor polygon to treat them as one [m]
    };

    /**
     * @brief Constructor
     */
    CostmapToSegmentedObjects():
      T(),
      dynamic_recfg_so_(nullptr)
    {
    }
       
    /**
     * @brief Destructor
     */
    virtual ~CostmapToSegmentedObjects()
    {
      if (dynamic_recfg_so_ != nullptr)
      {
        delete dynamic_recfg_so_;
      }
    }
    
    /**
     * @brief Initialize the plugin
     * @param nh Nodehandle that defines the namespace for parameters
     */
    virtual void initialize(ros::NodeHandle nh)
    {
      T::initialize(nh);

      // setup dynamic reconfigure
      // separate NH due to dynamic reconfigure services conflicts;
      // suffix meaning - Segmented Objects Dynamic Reconfigure
      dynamic_recfg_so_ = new dynamic_reconfigure::Server<CostmapToSegmentedObjectsConfig>(ros::NodeHandle(nh, nh.getNamespace() + "SODR"));
      dynamic_reconfigure::Server<CostmapToSegmentedObjectsConfig>::CallbackType cb = boost::bind(&CostmapToSegmentedObjects::reconfigureCB, this, _1, _2);
      dynamic_recfg_so_->setCallback(cb);
    }

    /**
     * @brief This method performs the actual work (conversion of the costmap to polygons)
     */
    virtual void compute()
    {
      // start with the computations performed by the base class
      T::compute();

      // Algorithm implemented below glues up polygons
      // that are close enough to each other (within segmentation_distance)

      // stores visited pairs of polygons
      std::vector<std::pair<size_t, size_t>> visited_pairs;
      // stores indices of pairs that can be glued together
      std::vector<std::pair<size_t, size_t>> extension_pairs;

      // let polygon IDs be counted from 0
      PolygonIdentified::resetPolygonsNum();

      // storage for polygons (initial set)
      std::vector<PolygonIdentified> polygons;
      auto polygons_input = *T::getPolygons();
      for (auto polygon : polygons_input) {
        polygons.push_back(polygon);
      }

      // 1st stage
      // this loop finds pairs of polygons whose closest vertices are close enough (below the threshold)
      for (const auto& polygon_a: polygons) {
        for (const auto& polygon_b: polygons) {
          // check if looking at self
          if (polygon_a.getID() == polygon_b.getID()) {
            continue;
          }

          // check if polygon pair was visited before (order of occurrence is not important)
          auto it_visited = std::find_if(
            visited_pairs.begin(),
            visited_pairs.end(),
            [&polygon_a, &polygon_b](const std::pair<size_t, size_t>& element) {
              return element.first == polygon_a.getID() && element.second == polygon_b.getID()
                || element.first == polygon_b.getID() && element.second == polygon_a.getID();
            }
          );

          // check if pair was already visited
          if (it_visited != visited_pairs.end()) {
            continue;
          }

          visited_pairs.push_back(std::make_pair(polygon_a.getID(), polygon_b.getID()));

          // flag that allows to abort further investigation for a given pair if pts close enough were already found
          bool found_extension_pair = false;
          // check each pair of points
          for (const auto& pt_a: polygon_a.getPolygon().points) {
            for (const auto& pt_b: polygon_b.getPolygon().points) {
              auto dist = std::hypot(pt_b.x - pt_a.x, pt_b.y - pt_a.y);
              if (dist <= parameter_so_.segmentation_distance_) {
                extension_pairs.push_back(std::make_pair(polygon_a.getID(), polygon_b.getID()));
                found_extension_pair = true;
                break;
              }
            }
            if (found_extension_pair)
            {
              break;
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
      for (size_t id = 0; id < PolygonIdentified::getPolygonsNum(); id++) {
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
          for (const auto& vertex: polygons.at(id).getPolygon().points) {
            tp.points.push_back(vertex);
          }
        }
        polygons_segmented->push_back(tp);
      }

      // NOTE: At this point we ended up with polygons that may contain excessive points (resultant objects may not be
      // convex hulls). Also, lines (initially) could have been transformed into polygons.
      // One may consider transforming glued up polygons to convex hulls (most probably the 'output' polygons contain
      // excessive points).

      // NOTE: must explicitly mark base class members here
      // update existing polygons set with the newly created one
      boost::mutex::scoped_lock lock(T::mutex_);
      T::polygons_ = polygons_segmented;
    }

    /**
     * @brief Get updated data from the previously set Costmap2D
     * @sa setCostmap2D
     */
    virtual void updateCostmap2D()
    {
      // get a copy of our parameters from dynamic reconfigure
      // `parameter_mutex_` cannot be used here
      boost::mutex::scoped_lock lock(parameter_so_mutex_);
      parameter_so_ = parameter_so_buffered_;

      T::updateCostmap2D();
    }

  protected:
    Parameters parameter_so_;          //< active parameters throughout computation
    Parameters parameter_so_buffered_; //< the buffered parameters that are offered to dynamic reconfigure
    boost::mutex parameter_so_mutex_;  //!< Mutex that keeps track about the ownership of the segmented objects params

    /**
     * @brief Callback for the dynamic_reconfigure node.
     * 
     * This callback allows to modify parameters dynamically at runtime without restarting the node
     * @param config Reference to the dynamic reconfigure config
     * @param level Dynamic reconfigure level
     */
    void reconfigureCB(CostmapToSegmentedObjectsConfig& config, uint32_t level)
    {
      boost::mutex::scoped_lock lock(parameter_so_mutex_);
      parameter_so_buffered_.segmentation_distance_ = config.segmentation_distance;
    }

    dynamic_reconfigure::Server<CostmapToSegmentedObjectsConfig>* dynamic_recfg_so_;
};

// explicitly name template class' types
typedef CostmapToSegmentedObjects<CostmapToPolygonsDBSMCCH> CostmapToSegmentedPolygonsDBSMCCH;
typedef CostmapToSegmentedObjects<CostmapToPolygonsDBSConcaveHull> CostmapToSegmentedPolygonsDBSConcaveHull;
typedef CostmapToSegmentedObjects<CostmapToLinesDBSRANSAC> CostmapToSegmentedLinesDBSRANSAC;
typedef CostmapToSegmentedObjects<CostmapToLinesDBSMCCH> CostmapToSegmentedLinesDBSMCCH;

} //end namespace costmap_converter
