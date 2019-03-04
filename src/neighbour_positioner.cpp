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
        local_position_pub_ = pnh_.advertise<geometry_msgs::TransformStamped>(local_position_topic_, 1);

        neighbours_sub_ = pnh_.subscribe<geometry_msgs::TransformStamped>(neighbours_topic_, 1,
                                                                          &NeighbourPositioner::neighboursCb, this);

        prohibition_layer_client_ = nh_.serviceClient<costmap_prohibition_layer::UpdateZones>(prohibition_layer_srv_);

        last_transform_ = tf::StampedTransform();
        last_tf_time_ = ros::Time(0);

        last_neighbour_transform_ = geometry_msgs::TransformStamped();
        last_time_neighbour_ = ros::Time(0);
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

    std::string local_position_topic = "position_transform";
    readParam(pnh_, "local_position_topic", local_position_topic_, local_position_topic, true);

    std::string fixed_frame = "map";
    readParam(pnh_, "fixed_frame", fixed_frame_, fixed_frame, true);

    std::string base_frame = "base_footprint";
    readParam(pnh_, "base_frame", base_frame_, base_frame, true);

    std::string neighbours_topic = "neighbours";
    readParam(pnh_, "neighbours_topic", neighbours_topic_, neighbours_topic, true);

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
        RCOMPONENT_ERROR("Couldn't get the transform betwen fixed_frame and base_frame");
    }

    if (((ros::Time::now() - last_tf_time_).toSec() > ros::Duration(2.0).toSec()) == false)
    {
        geometry_msgs::TransformStamped transform;
        transformStampedTFToMsg(current_transform_, transform);
        local_position_pub_.publish(transform);
    }
    else
    {
        RCOMPONENT_ERROR("Too much time without tf between base_frame and fixed_frame");
    }

    // PUBLICATION OF THE NEIGHBOURS POSITIONS AS A PROHIBITION AREA
    if (((ros::Time::now() - last_time_neighbour_).toSec() > ros::Duration(2.0).toSec() == false))
    {
        double diff_x = current_neighbour_transform_.transform.translation.x -
                        last_neighbour_transform_.transform.translation.x;
        double diff_y = current_neighbour_transform_.transform.translation.y -
                        last_neighbour_transform_.transform.translation.y;
        double dist = sqrt(diff_x * diff_x + diff_y * diff_y);
        if (dist > 0.05)
        {
            // TODO: SET THE FOOTPRINT OF THE OTHER ROBOT AS OBSTACLE
            // CREATE THE OBSTACLE
            std::vector<geometry_msgs::Point> vector_to_add;
            geometry_msgs::Point point;

            double offset = 0.2;
            double rotation = tf::getYaw(current_neighbour_transform_.transform.rotation);
            point.x = current_neighbour_transform_.transform.translation.x + offset;
            point.y = current_neighbour_transform_.transform.translation.y + offset / 5;
            point.x = point.x * cos(rotation) - point.y * sin(rotation);
            point.y = point.x * sin(rotation) + point.y * cos(rotation);
            vector_to_add.push_back(point);
            point.x = current_neighbour_transform_.transform.translation.x + offset;
            point.y = current_neighbour_transform_.transform.translation.y - offset / 5;
            point.x = point.x * cos(rotation) - point.y * sin(rotation);
            point.y = point.x * sin(rotation) + point.y * cos(rotation);
            vector_to_add.push_back(point);
            point.x = current_neighbour_transform_.transform.translation.x - offset;
            point.y = current_neighbour_transform_.transform.translation.y - offset;
            point.x = point.x * cos(rotation) - point.y * sin(rotation);
            point.y = point.x * sin(rotation) + point.y * cos(rotation);
            vector_to_add.push_back(point);
            point.x = current_neighbour_transform_.transform.translation.x - offset;
            point.y = current_neighbour_transform_.transform.translation.y + offset;
            point.x = point.x * cos(rotation) - point.y * sin(rotation);
            point.y = point.x * sin(rotation) + point.y * cos(rotation);
            vector_to_add.push_back(point);

            costmap_prohibition_layer::UpdateZones update_zones;
            update_zones.request.zone = vector_to_add;
            prohibition_layer_client_.call(update_zones);

            last_neighbour_transform_ = current_neighbour_transform_;
            RCOMPONENT_WARN("Update prohibition layer");
        }
    }
    else
    {
        RCOMPONENT_ERROR("Too much time without receiving neighbours postion");
    }

    ros::Duration(1 / desired_freq_).sleep();
}

void NeighbourPositioner::emergencyState()
{
}

void NeighbourPositioner::failureState()
{
}

void NeighbourPositioner::neighboursCb(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    current_neighbour_transform_ = *msg;
    last_time_neighbour_ = ros::Time::now();
}