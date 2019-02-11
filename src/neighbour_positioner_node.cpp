#include <neighbour_positioner/neighbour_positioner.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "neighbour_positioner");
    ros::NodeHandle n;

    NeighbourPositioner neighbour_positioner(n);
    neighbour_positioner.start();
}