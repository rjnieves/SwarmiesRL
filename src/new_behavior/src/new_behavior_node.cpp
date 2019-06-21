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
#include "SystemStateSample.hh"

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
        SystemStateSample< 3u > aState;

        nodeConfig.addCommandLineConfig(argc, argv);

        if (nodeConfig.helpRequested())
        {
            nodeConfig.printCommandLineHelpTo(cout);
        }
        else
        {
            stringstream nodeName;
            nodeName << nodeConfig.robotName() << "_NEW_BEHAVIOR";
            cout << "Current State:" << aState << endl;
            double *stateVals = new double[aState.size()];
            std::copy(aState.begin(), aState.end(), stateVals);
            std::copy(stateVals, stateVals + aState.size(), std::ostream_iterator< double >(cout, " "));
            cout << endl;
            delete [] stateVals;
            stateVals = nullptr;

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
