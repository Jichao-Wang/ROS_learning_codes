#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <wjc_pk/CustomMsg.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char **argv)
{
    ros::init(argc, argv, "writer_example");
    ros::NodeHandle n;

    /*an example for getting time*/
    /*ros::Time begin=ros::Time::now();
    std::cout<<begin<<std::endl;*/

    /*an example for reading a bag included two fields per view*/
    rosbag::Bag bag;
    bag.open("wjc_test.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("chatter"));
    topics.push_back(std::string("numbers"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    foreach(rosbag::MessageInstance const m, view)
    {
        std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
        if (s != NULL)
            std::cout << s->data << std::endl;
            
        std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
        if (i != NULL)
            std::cout << i->data << std::endl;
    }
    bag.close();
}