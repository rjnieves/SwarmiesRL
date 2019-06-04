/**
 * \file new_behavior_node.cpp
 * \brief Contains the \c main() function for the \c new_behavior_node ROS node.
 * \date 2019-06-04 16:02:16
 * \author Rolando J. Nieves
 */

#include <cstdlib>
#include <string>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <ros/ros.h>

#include "NodeConfiguration.hh"

using std::cout;
using std::cerr;
using std::endl;
using std::stringstream;
using std::exception;

int main(int argc, char const **argv)
{
    int result = EXIT_SUCCESS;

    try
    {
        NodeConfiguration nodeConfig;

        nodeConfig.addCommandLineConfig(argc, argv);

        if (nodeConfig.helpRequested())
        {
            nodeConfig.printCommandLineHelpTo(cout);
        }
        else
        {
            stringstream nodeName;
            nodeName << nodeConfig.robotName() << "_NEW_BEHAVIOR";

            ros::init(argc, const_cast< char** >(argv), nodeName.str());
            ros::spin();
        }
    }
    catch(exception& ex)
    {
        cerr << "Exception raised:" << ex.what() << endl;
        result = EXIT_FAILURE;
    }

    return result;
}

// vim: set ts=4 sw=4 expandtab:
