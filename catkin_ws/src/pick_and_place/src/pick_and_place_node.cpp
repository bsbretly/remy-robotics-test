#include "pick_and_place/pick_and_place.hpp"

int main(int argc, char** argv) {

    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle nh("~");
    PickAndPlace pickAndPlace(nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    pickAndPlace.pickNPlace();
    ros::waitForShutdown();

    return 0;
}