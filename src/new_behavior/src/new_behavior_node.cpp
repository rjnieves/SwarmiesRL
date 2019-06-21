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
#include <algorithm>

#include <ros/ros.h>

#include "NodeConfiguration.hh"
#include "SystemStateRepository.hh"

using std::cout;
using std::cerr;
using std::endl;
using std::stringstream;
using std::exception;

using ThreeRobotSystemState = SystemStateRepository< 3u >;

int main(int argc, char const **argv)
{
    int result = EXIT_SUCCESS;

    try
    {
        NodeConfiguration nodeConfig;
        ThreeRobotSystemState sysState {0.5, 16u};

        nodeConfig.addCommandLineConfig(argc, argv);

        if (nodeConfig.helpRequested())
        {
            nodeConfig.printCommandLineHelpTo(cout);
        }
        else
        {
            stringstream nodeName;
            nodeName << nodeConfig.robotName() << "_NEW_BEHAVIOR";
            cout << "Current State:" << sysState.currentState() << endl;
            sysState.roverAtPos(0u, 0.0, 1.308);
            sysState.roverAtPos(1u, -1.308, 0.0);
            sysState.roverAtPos(2u, 0.0, -1.308);
            cout << "Current State:" << sysState.currentState() << endl;
            sysState.foundBlockAt(-3.0, -2.8);
            sysState.roverAtPos(0u, 0.0, 1.308);
            sysState.roverAtPos(1u, -1.308, 0.0);
            sysState.roverAtPos(2u, 0.0, -1.308);
            cout << "Current State:" << sysState.currentState() << endl;
            sysState.roverAtPos(0u, 0.0, 1.308);
            sysState.roverAtPos(1u, -1.308, 0.0);
            sysState.roverAtPos(2u, -0.20, -1.308);
            cout << "Current State:" << sysState.currentState() << endl;
            sysState.roverAtPos(0u, 0.0, 1.308);
            sysState.roverAtPos(1u, -1.308, 0.0);
            sysState.roverAtPos(2u, -0.40, -1.308);
            cout << "Current State:" << sysState.currentState() << endl;
            sysState.roverAtPos(0u, 0.0, 1.308);
            sysState.roverAtPos(1u, -1.308, 0.0);
            sysState.roverAtPos(2u, -0.60, -1.308);
            cout << "Current State:" << sysState.currentState() << endl;
            sysState.roverAtPos(0u, 0.0, 1.308);
            sysState.roverAtPos(1u, -1.308, 0.0);
            sysState.roverAtPos(2u, -0.80, -1.308);
            cout << "Current State:" << sysState.currentState() << endl;
            sysState.roverAtPos(0u, 0.0, 1.308);
            sysState.roverAtPos(1u, -1.308, 0.0);
            sysState.roverAtPos(2u, -1.00, -1.308);
            cout << "Current State:" << sysState.currentState() << endl;

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
