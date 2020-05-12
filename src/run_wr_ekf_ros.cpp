#include <iostream>
#include "WrEkfNode.h"
#include <signal.h>


WrEkfNode* wrEkfNode;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "wr_ekf_ros");
    wrEkfNode = new WrEkfNode();
    std::cout << "wr_ekf_node started\n";
    wrEkfNode->run();
    return 0;
}
