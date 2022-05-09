#pragma once

#include <costmap_converter/costmap_to_polygons.h>

// dynamic reconfigure
#include <costmap_converter/CostmapToSegmentedObjectsConfig.h>
#include <dynamic_reconfigure/server.h>

namespace costmap_converter
{
/**
 * @class CostmapToSegmentedObjects
 */
class CostmapToSegmentedObjects : public CostmapToPolygonsDBSMCCH {

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
    CostmapToSegmentedObjects();
       
    /**
     * @brief Destructor
     */
    virtual ~CostmapToSegmentedObjects();
    
    /**
     * @brief Initialize the plugin
     * @param nh Nodehandle that defines the namespace for parameters
     */
    virtual void initialize(ros::NodeHandle nh);

    /**
     * @brief This method performs the actual work (conversion of the costmap to polygons)
     */
    virtual void compute();

    /**
     * @brief Get updated data from the previously set Costmap2D
     * @sa setCostmap2D
     */
    virtual void updateCostmap2D();

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
    void reconfigureCB(CostmapToSegmentedObjectsConfig& config, uint32_t level);

    dynamic_reconfigure::Server<CostmapToSegmentedObjectsConfig>* dynamic_recfg_so_;
};

} //end namespace costmap_converter
