/**
 * \file NodeConfiguration.cpp
 * \brief Contains the implementation of the \c NodeConfiguration class.
 * \date 2019-06-04 17:24:41
 * \author Rolando J. Nieves
 */

#include <regex>
#include <stdexcept>
#include <sstream>

#include "NodeConfiguration.hh"


using std::string;
using std::regex;
using std::ostream;
using std::runtime_error;
using std::stringstream;
using std::endl;

namespace po = boost::program_options;

namespace
{

const string HELP_FLAG { "help" };
const string ROBOT_NAME_FLAG { "robot-name" };
const regex VALID_ROBOT_NAME_PATTERN { "\\w+" };

} // end private namespace


NodeConfiguration::NodeConfiguration():
    m_helpRequested(false)
{
    m_basicOptions.add_options()
        (HELP_FLAG.c_str(), "Produce this help message.")
    ;

    m_hiddenOptions.add_options()
        (ROBOT_NAME_FLAG.c_str(), po::value< string >(), "Name for the robot.")
    ;
}


NodeConfiguration::~NodeConfiguration()
{

}


void
NodeConfiguration::addCommandLineConfig(int argc, char const **argv)
{
    po::positional_options_description positionalOpts;
    positionalOpts.add(ROBOT_NAME_FLAG.c_str(), 1);

    po::options_description allOptions;
    allOptions.add(m_basicOptions).add(m_hiddenOptions);

    po::variables_map cmdLineConfig;
    po::store(
        po::command_line_parser(
            argc,
            argv
        )
        .options(allOptions)
        .positional(positionalOpts)
        .run(),
        cmdLineConfig
    );
    po::notify(cmdLineConfig);

    m_helpRequested = cmdLineConfig.count(HELP_FLAG);

    if (cmdLineConfig.count(ROBOT_NAME_FLAG))
    {
        m_robotName = cmdLineConfig[ROBOT_NAME_FLAG].as< string >();
        bool validName = std::regex_match(
            m_robotName,
            VALID_ROBOT_NAME_PATTERN
        );
        if (!validName)
        {
            stringstream errorMsg;
            errorMsg << "Invalid robot name \"" << m_robotName << "\"";
            m_robotName.clear();
            throw runtime_error(errorMsg.str());
        }
    }
}


void
NodeConfiguration::printCommandLineHelpTo(ostream& stream)
{
    stream << "Usage: new_behavior_node [options] <robot-name>" << endl;
    stream << m_basicOptions << endl;
}

// vim: set ts=4 sw=4 expandtab:
