#include "filter.h"

Filter::Filter(ros::NodeHandle &nh)
{
    EM_sub_ = nh.subscribe("/em_odometry", 10, &Filter::EMCallback, this);
}

Filter::~Filter()
{
}

void Filter::EMCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    // do something
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "em_filter");
    ros::NodeHandle nh;

    Filter filter_(nh);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}