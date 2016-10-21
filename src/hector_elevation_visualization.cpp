#include <hector_elevation_visualization/hector_elevation_visualization.h>


// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))


ElevationVisualization::ElevationVisualization(ros::NodeHandle& nHandle)
{
    ros::NodeHandle nHandle_private("~");
    nHandle_private.param("max_height_levels",max_height_levels,10); //[cell]
    nHandle_private.param("min_height",min_height,-1.0); //[m]
    nHandle_private.param("max_height",max_height,1.5); //[m]
    nHandle_private.param("color_factor",color_factor,0.8); //[]
    double r, g, b, a;
    nHandle_private.param("color/r", r, 0.0);
    nHandle_private.param("color/g", g, 0.0);
    nHandle_private.param("color/b", b, 1.0);
    nHandle_private.param("color/a", a, 1.0);
    marker_color.r = r;
    marker_color.g = g;
    marker_color.b = b;
    marker_color.a = a;
    nHandle_private.param("use_color_map", use_color_map, true); //[]

    nHandle_private.param("elevation_map_frame_id", elevation_map_frame_id,std::string("/elevation_map_local"));
    // Added by Mohsen
    resolution_xy = 0.05;
    origin_x = 0.493;//-1
    origin_y = -2.5;//-3
    costmap_x_size = 5.0;
    costmap_y_size = 5.0;
    nHandle_private.param("map_frame_id", map_frame_id,std::string("/map"));
    nHandle_private.param("base_frame_id", base_frame_id,std::string("/base_link"));
    // Adding the costmap 
    
    nHandleP = &nHandle;
    ROS_WARN("Elevation costmap is initialized!");
    elevation_topic_name = "/elevation_costmap";

    cell_elevation_x = floor (costmap_x_size/resolution_xy);
    cell_elevation_y = floor (costmap_y_size/resolution_xy);
    //ROS_INFO_ONCE("cell_elevation_x: %d,   cell_elevation_y: %d   ", cell_elevation_x, cell_elevation_y);


    elev_map.header.frame_id = std::string("base_link");
    elev_map.info.resolution = resolution_xy;
    elev_map.info.width = cell_elevation_x;
    elev_map.info.height = cell_elevation_y;
    elev_map.info.origin.position.x = origin_x;
    elev_map.info.origin.position.y = origin_y;
    elev_map.info.origin.position.z = 0.0;
    elev_map.info.origin.orientation.x = 0.0;
    elev_map.info.origin.orientation.y = 0.0;
    elev_map.info.origin.orientation.z = 0.0;
    elev_map.info.origin.orientation.w = 1.0;
    elev_map.data = std::vector<int8_t> (cell_elevation_x*cell_elevation_y);

    Init_map();
    //End
    nHandle_private.param("paramSysMsgTopic", sys_msg_topic, std::string("/syscommand"));

    map_marker_array_publisher = nHandle.advertise<visualization_msgs::MarkerArray>("elevation_map_marker_array", 1,true);

    Map_publisher = nHandle.advertise<nav_msgs::OccupancyGrid>("/elevation_costmap", 1,true);
    EcostmapMeta_publisher = nHandle.advertise<hector_elevation_visualization::EcostmapMetaData>("/elevation_costmap_MetaData", 1);

    sub_elevation_map = nHandle.subscribe(elevation_map_frame_id,1,&ElevationVisualization::map_callback,this);
    sub_sys_message_callback = nHandle.subscribe(sys_msg_topic,1,&ElevationVisualization::sys_message_callback,this);
    sub_CloudMetaData = nHandle.subscribe("RL_cloud_MetaData",1,&ElevationVisualization::CloudMetaData_cb,this);


    dyn_rec_server_.setCallback(boost::bind(&ElevationVisualization::dynRecParamCallback, this, _1, _2));

    ros::spin();
}

ElevationVisualization::~ElevationVisualization()
{

}

void ElevationVisualization::dynRecParamCallback(hector_elevation_visualization::ElevationVisualizationConfig &config, uint32_t level)
{
    max_height_levels = config.max_height_levels;
    min_height = config.min_height;
    max_height = config.max_height;
    color_factor = config.color_factor;
    use_color_map = config.use_color_map;
    
    if(std::strcmp(elevation_map_frame_id.c_str(),(config.elevation_map_frame_id).c_str()))
    {
        elevation_map_frame_id = config.elevation_map_frame_id;
	
        sub_elevation_map = nHandle.subscribe(elevation_map_frame_id,1,&ElevationVisualization::map_callback,this);
    }
}


void ElevationVisualization::visualize_map(const hector_elevation_msgs::ElevationGrid& elevation_map, tf::StampedTransform local_map_transform)
{
    
    int current_height_level;
    double elevation_cost;
    double height_max = 0.5;
    double height_min = -3.0;
    unsigned int index_x_cell;
    unsigned int index_y_cell;
    // Updating OccupancyGrid MetaData
    elev_map.info.width = cell_elevation_x;
    elev_map.info.height = cell_elevation_y;

    elev_map.info.origin.position.x = origin_x;
    elev_map.info.origin.position.y = origin_y;

    // EcostmapMetaData msg
    ecostmap_meta.elevation_min = elevation_map.info.min_elevation;
    ecostmap_meta.elevation_max = elevation_map.info.max_elevation;
    ecostmap_meta.cost_resolution = 254.00/(ecostmap_meta.elevation_max - ecostmap_meta.elevation_min);
    ecostmap_meta.zero_elevation_cost = fabs(ecostmap_meta.elevation_min*ecostmap_meta.cost_resolution);
    ecostmap_meta.origin_x = origin_x;
    ecostmap_meta.origin_y = origin_y; 
    ecostmap_meta.x_size = costmap_x_size;
    ecostmap_meta.y_size = costmap_y_size;


    int x_offset = 10; //Distance between laser and base_link
    int8_t map_cost;
   
    

    if (fabs(costmap_x_size - costmap_y_size)>0.001)
    {
        ROS_ERROR("The Point Cloud has to have square bounds in x-y plane, Message is Skipped");
        ROS_WARN("/elevation_costmap will not being published!");
        return;
    }
    for (int index_y = 0; index_y < (int)elevation_map.info.height; ++index_y)
    {
	
        //index_y_cell = index_y - 511 + floor(cell_elevation_y/2);
	    if ( (index_y - 511 + floor(cell_elevation_y/2)) > 0)
        	index_y_cell = index_y - 511 + floor(cell_elevation_y/2);
	    else
		    index_y_cell = 1;
	    //debug
	
	    if (index_y_cell > cell_elevation_y) index_y_cell = cell_elevation_y;
        for (int index_x = 0; index_x < (int)elevation_map.info.width; ++index_x)
        {
            // visualize only known cells
	    
            if(elevation_map.data[MAP_IDX(elevation_map.info.width, index_x, index_y)] != (int16_t)-elevation_map.info.zero_elevation)
            //if(true)
	        {
                geometry_msgs::Point cube_center;
                Eigen::Vector2f index_map(index_x, index_y);
                Eigen::Vector2f index_world = world_map_transform.getC1Coords(index_map);

                cube_center.x = index_world(0);//+elevation_map.info.resolution_xy/2.0;
                cube_center.y = index_world(1);//+elevation_map.info.resolution_xy/2.0;
                cube_center.z = (elevation_map.data[MAP_IDX(elevation_map.info.width, index_x, index_y)]-elevation_map.info.zero_elevation)*elevation_map.info.resolution_z;
		
		        elevation_cost = floor ( 255/(height_max - height_min)*(cube_center.z - height_min) );
		        //ROS_INFO("elevation_cost = %f",elevation_cost);
		        elevation_cost = std::min(elevation_cost,254.0);
		        elevation_cost = std::max(elevation_cost,0.0);
		        map_cost = (int8_t) floor(elevation_cost/254*100);
		        //ROS_INFO("map cost = %d",map_cost);
		
		

		
		        if (index_x - 511 - x_offset > 0)
			        index_x_cell = index_x - 511 - x_offset;
		        else
			        index_x_cell = 1;
		        if (index_x_cell > cell_elevation_x) index_x_cell = cell_elevation_x;
		
		
                if ((0 < index_x_cell) && (index_x_cell <= cell_elevation_x) && (0 < index_y_cell) && (index_y_cell <= cell_elevation_y))
		        {
			        setCost(index_x_cell,index_y_cell,map_cost);			
		        }
		
            }
        }
	
    }
    elev_map.header.stamp = ros::Time::now();
	elev_map.header.frame_id = std::string("/base_link");
	Map_publisher.publish(elev_map);
    EcostmapMeta_publisher.publish(ecostmap_meta);

}
void ElevationVisualization::CloudMetaData_cb(const pc_maker::CloudMetaData::Ptr msg)
{
    cell_elevation_x = floor (msg->x_max/resolution_xy);
    cell_elevation_y = floor (2*(msg->y_max)/resolution_xy);
    if(fabs( costmap_x_size - msg->x_max) > 0.001)
    {
        elev_map.data = std::vector<int8_t> (cell_elevation_x*cell_elevation_y);
        // Initializing OccupancyGrid with "unkown value"" (-1)
        Init_map();
    }
    costmap_x_size = msg->x_max;
    costmap_y_size = 2*(msg->y_max);

    origin_y = -msg->y_max;
    elev_map.info.width = cell_elevation_x;
    elev_map.info.height = cell_elevation_y;
    elev_map.info.origin.position.x = origin_x;
    elev_map.info.origin.position.y = origin_y;
   
}

void ElevationVisualization::map_callback(const hector_elevation_msgs::ElevationGrid& elevation_map)
{
    //ROS_ERROR("height: %f, width: %f, res: %f",elevation_map.info.height,elevation_map.info.width,elevation_map.info.resolution_xy);
    // Visualization of Elevation Map
    
        tf::TransformListener listener;
        tf::StampedTransform local_map_transform;

        // get local map transform
        try
        {
            listener.waitForTransform(base_frame_id, map_frame_id,ros::Time(0),ros::Duration(5.0));
            listener.lookupTransform(base_frame_id, map_frame_id, ros::Time(0), local_map_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }

        // init transform
        nav_msgs::MapMetaData map_meta;
        map_meta.resolution = elevation_map.info.resolution_xy;
        map_meta.height = elevation_map.info.height;
        map_meta.width = elevation_map.info.width;
        map_meta.origin = elevation_map.info.origin;

        world_map_transform.setTransforms(map_meta);

        // visualize map
        visualize_map(elevation_map, local_map_transform);

	//Publish costmap
	//elevation_grid_ros->publishCostmap();

        // publish map
        //map_marker_array_publisher.publish(map_marker_array_msg);
    
}

void ElevationVisualization::sys_message_callback(const std_msgs::String& string)
{
    ROS_DEBUG("sysMsgCallback, msg contents: %s", string.data.c_str());

    if (string.data == "reset")
    {
        ROS_INFO("reset");

        for (unsigned i = 0; i < map_marker_array_msg.markers.size(); ++i)
        {
            map_marker_array_msg.markers[i].points.clear();
        }
        map_marker_array_msg.markers.clear();
    }
}

void ElevationVisualization::setCost(unsigned int i, unsigned int j, int8_t cost)
{
	int index = i + cell_elevation_y*j;
	//ROS_INFO("index: %d",index);
 	if(index > 0 && index < (cell_elevation_x * cell_elevation_y) )// (costmap_y_size * costmap_x_size)
  		elev_map.data[index] = cost;
	//else
	//	ROS_ERROR("Undisred index: %d", index);
}

void ElevationVisualization::Init_map()
{
	for(int i = 0; i < elev_map.info.width * elev_map.info.height; i++)
		elev_map.data[i] = -1;
}

std_msgs::ColorRGBA ElevationVisualization::heightMapColor(double h)
{

    std_msgs::ColorRGBA color;
    color.a = 1.0;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1))
        f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
    case 6:
    case 0:
        color.r = v; color.g = n; color.b = m;
        break;
    case 1:
        color.r = n; color.g = v; color.b = m;
        break;
    case 2:
        color.r = m; color.g = v; color.b = n;
        break;
    case 3:
        color.r = m; color.g = n; color.b = v;
        break;
    case 4:
        color.r = n; color.g = m; color.b = v;
        break;
    case 5:
        color.r = v; color.g = m; color.b = n;
        break;
    default:
        color.r = 1; color.g = 0.5; color.b = 0.5;
        break;
    }

    return color;
}



