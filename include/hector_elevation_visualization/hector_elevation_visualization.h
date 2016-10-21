#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_msgs/String.h>

#include <hector_elevation_msgs/ElevationMapMetaData.h>
#include <hector_elevation_msgs/ElevationGrid.h>

#include <dynamic_reconfigure/server.h>
#include <hector_elevation_visualization/ElevationVisualizationConfig.h>

#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Core>

#include <hector_map_tools/HectorMapTools.h>

#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>

#include <nav_msgs/OccupancyGrid.h>

#include <pc_maker/CloudMetaData.h>

#include <hector_elevation_visualization/EcostmapMetaData.h>

class ElevationVisualization{

public:
    /// Default constructor
    ElevationVisualization(ros::NodeHandle& nHandle);

    /// Default deconstructor
    ~ElevationVisualization();

    /// dynRecParamCallback This function get called if new parameters has been set with the dynamic reconfigure dialog
    /**
    * \param [in] config contains current parameters
    * \param [in] level is unused
    */

    void dynRecParamCallback(hector_elevation_visualization::ElevationVisualizationConfig &config, uint32_t level);

    /// sysMessageCallback This function listen to system messages
    /**
    * \param [in] string parameter contains system messages, like "reset"
    */
    void sys_message_callback(const std_msgs::String& string);

    /// map_callback get called if a new elevation map is avaible
    /**
    * \param [in] elevation_map_msg stores elevation map data as a 2.5D grid
    */
    void CloudMetaData_cb(const pc_maker::CloudMetaData::Ptr msg);
    void map_callback(const hector_elevation_msgs::ElevationGrid& elevation_map);

private:
    ros::NodeHandle nHandle;
    ros::NodeHandle* nHandleP;

    ros::Subscriber sub_elevation_map;
    ros::Subscriber sub_sys_message_callback;
    ros::Subscriber sub_CloudMetaData; 

    ros::Publisher map_marker_array_publisher;
    ros::Publisher Map_publisher;
    ros::Publisher EcostmapMeta_publisher;

    dynamic_reconfigure::Server<hector_elevation_visualization::ElevationVisualizationConfig> dyn_rec_server_;

    visualization_msgs::MarkerArray map_marker_array_msg;

    HectorMapTools::CoordinateTransformer<float> world_map_transform;

    std::string elevation_map_frame_id,sys_msg_topic,map_frame_id,base_frame_id;

    int max_height_levels;

    double min_height,max_height,color_factor;

    bool use_color_map;

    std_msgs::ColorRGBA marker_color;

    /// visualize_map calculates visualization markers to vizualize the elevation map in rviz
    /**
    * \param [in] elevation_map elevation map data as a 2.5D grid
    * \param [in] local_map_transform is used for deducing the robot's position
    */
    void visualize_map(const hector_elevation_msgs::ElevationGrid& elevation_map, tf::StampedTransform local_map_transform);

    /// heightMapColor calculates the marker color as a function of height
    /**
    * \param [in] h The height in [m]
    */
    static std_msgs::ColorRGBA heightMapColor(double h);

protected:
    void setCost(unsigned int x, unsigned int y, int8_t cost);
    void Init_map();
    //costmap_2d::Costmap2D* elevation_grid_;
    //costmap_2d::Costmap2DPublisher* elevation_grid_ros;
    double resolution_xy;
    double origin_x;
    double origin_y;
    double costmap_x_size;
    double costmap_y_size;
    unsigned int cell_elevation_x;
    unsigned int cell_elevation_y;
    std::string elevation_topic_name;
    nav_msgs::OccupancyGrid elev_map;
    hector_elevation_visualization::EcostmapMetaData ecostmap_meta;

};
