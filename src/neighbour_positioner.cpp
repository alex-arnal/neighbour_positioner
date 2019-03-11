#include <neighbour_positioner/neighbour_positioner.h>

NeighbourPositioner::NeighbourPositioner(ros::NodeHandle h) : RComponent(h), nh_(h), pnh_("~")
{
    rosReadParams();
}

NeighbourPositioner::~NeighbourPositioner()
{
}

int NeighbourPositioner::rosSetup()
{
    if (RComponent::rosSetup() == rcomponent::OK)
    {
        local_position_pub_ = pnh_.advertise<neighbour_positioner::NeighbourLocation>(neighbours_topic_, 1);

        neighbours_sub_ =
            pnh_.subscribe<neighbour_positioner::NeighbourLocation>(neighbours_topic_, 1,
                                                                    &NeighbourPositioner::neighboursCb, this);

        prohibition_layer_client_ = nh_.serviceClient<costmap_prohibition_layer::UpdateZones>(prohibition_layer_srv_);

        last_tf_time_ = ros::Time(0);
    }
}

int NeighbourPositioner::rosShutdown()
{
    RComponent::rosShutdown();
}

void NeighbourPositioner::rosPublish()
{
    RComponent::rosPublish();
}

void NeighbourPositioner::rosReadParams()
{
    double desired_freq = DEFAULT_THREAD_DESIRED_HZ;
    readParam(pnh_, "desired_freq", desired_freq_, desired_freq_, false);

    std::string fixed_frame = "map";
    readParam(pnh_, "fixed_frame", fixed_frame_, fixed_frame, true);

    std::string base_frame = "base_footprint";
    readParam(pnh_, "base_frame", base_frame_, base_frame, true);

    std::string neighbours_topic = "neighbours";
    readParam(pnh_, "neighbours_topic", neighbours_topic_, neighbours_topic, true);

    std::string robot_name = "robot";
    readParam(pnh_, "robot_name", robot_name_, robot_name, true);

    std::string prohibition_layer_srv = "update_zones";
    readParam(pnh_, "prohibition_layer_srv", prohibition_layer_srv_, prohibition_layer_srv, true);
}

void NeighbourPositioner::standbyState()
{
    switchToState(robotnik_msgs::State::READY_STATE);
}
void NeighbourPositioner::readyState()
{
    // LOCAL TRANSFORM FROM FIXED_FRAME TO BASE_FRAME PUBLICATION
    try
    {
        tf_listener_.waitForTransform(fixed_frame_, base_frame_, ros::Time(0), ros::Duration(2));
        tf_listener_.lookupTransform(fixed_frame_, base_frame_, ros::Time(0), current_transform_);
        last_tf_time_ = ros::Time::now();
    }
    catch (tf2::LookupException error)
    {
        RCOMPONENT_ERROR_THROTTLE(2, "Couldn't get the transform betwen %s (fixed_frame) and %s (base_frame)",
                                  fixed_frame_.c_str(), base_frame_.c_str());
    }

    if (((ros::Time::now() - last_tf_time_).toSec() > ros::Duration(2.0).toSec()) == false)
    {
        neighbour_positioner::NeighbourLocation location;
        transformStampedTFToMsg(current_transform_, location.transform);
        location.name = robot_name_;
        local_position_pub_.publish(location);
    }
    else
    {
        RCOMPONENT_ERROR_THROTTLE(2, "Too much time without tf between %s (fixed_frame) and %s (base_frame)",
                                  fixed_frame_.c_str(), base_frame_.c_str());
    }

    std::map<string, neighbour_info>::iterator it;
    std::vector<geometry_msgs::Polygon> polygons;
    bool need_update = false;

    // PUBLICATION OF THE NEIGHBOURS POSITIONS AS A PROHIBITION AREA
    for (it = neighbourhood.begin(); it != neighbourhood.end(); it++)
    {
        string neighbour_name = it->first;
        geometry_msgs::TransformStamped current_neighbour_transform = it->second.current_location;
        geometry_msgs::TransformStamped last_neighbour_transform = it->second.last_location;
        ros::Time last_time_neighbour = it->second.last_time;
        if (((ros::Time::now() - last_time_neighbour).toSec() > ros::Duration(2.0).toSec() == false))
        {

            double diff_x = current_neighbour_transform.transform.translation.x -
                            last_neighbour_transform.transform.translation.x;
            double diff_y = current_neighbour_transform.transform.translation.y -
                            last_neighbour_transform.transform.translation.y;
            double dist = sqrt(diff_x * diff_x + diff_y * diff_y);

            geometry_msgs::Polygon polygon;
            geometry_msgs::Point32 point;

            double offset = 0.2;
            if (dist > 0.05)
            {
                need_update = true;
                // TODO: SET THE FOOTPRINT OF THE OTHER ROBOT AS OBSTACLE
                // CREATE THE OBSTACLE
                point.x = current_neighbour_transform.transform.translation.x + offset;
                point.y = current_neighbour_transform.transform.translation.y + offset / 5;
                polygon.points.push_back(point);
                point.x = current_neighbour_transform.transform.translation.x + offset;
                point.y = current_neighbour_transform.transform.translation.y - offset / 5;
                polygon.points.push_back(point);
                point.x = current_neighbour_transform.transform.translation.x - offset;
                point.y = current_neighbour_transform.transform.translation.y - offset;
                polygon.points.push_back(point);
                point.x = current_neighbour_transform.transform.translation.x - offset;
                point.y = current_neighbour_transform.transform.translation.y + offset;
                polygon.points.push_back(point);

                polygons.push_back(polygon);
                it->second.last_location = it->second.current_location;
            }
            else
            {
                point.x = last_neighbour_transform.transform.translation.x + offset;
                point.y = last_neighbour_transform.transform.translation.y + offset / 5;
                polygon.points.push_back(point);
                point.x = last_neighbour_transform.transform.translation.x + offset;
                point.y = last_neighbour_transform.transform.translation.y - offset / 5;
                polygon.points.push_back(point);
                point.x = last_neighbour_transform.transform.translation.x - offset;
                point.y = last_neighbour_transform.transform.translation.y - offset;
                polygon.points.push_back(point);
                point.x = last_neighbour_transform.transform.translation.x - offset;
                point.y = last_neighbour_transform.transform.translation.y + offset;
                polygon.points.push_back(point);

                polygons.push_back(polygon);
            }
        }
        else
        {
            RCOMPONENT_ERROR_THROTTLE(2, "Too much time without receiving neighbour %s postion",
                                      neighbour_name.c_str());
        }
    }

    if (need_update == true)
    {
        costmap_prohibition_layer::UpdateZones update_zones;
        update_zones.request.zones = polygons;
        prohibition_layer_client_.call(update_zones);
        RCOMPONENT_WARN("Update prohibition layer");
    }

    ros::Duration(1 / desired_freq_).sleep();
}

void NeighbourPositioner::emergencyState()
{
}

void NeighbourPositioner::failureState()
{
}

void NeighbourPositioner::neighboursCb(const neighbour_positioner::NeighbourLocation::ConstPtr &msg)
{
    if (msg->name != robot_name_)
    {
        neighbour_info neighbour;
        neighbour.last_time = ros::Time::now();
        neighbour.current_location = msg->transform;
        neighbour.last_location = msg->transform;
        std::map<string, neighbour_info>::iterator it = neighbourhood.find(msg->name);
        // The neignour already exists. We just update the info.
        if (it != neighbourhood.end())
        {
            neighbour.last_location = it->second.last_location;
            it->second = neighbour;
        }
        // New neighbour has been located.
        else
        {
            neighbourhood.insert(pair<string, neighbour_info>(msg->name, neighbour));
            RCOMPONENT_INFO("New neighbour (%s) located", msg->name.c_str());
        }
    }
}