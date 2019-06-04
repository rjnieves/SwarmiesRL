/**
 * \file NodeConfiguration.hh
 * \brief Contains the definition for the \c NodeConfiguration class.
 * \date 2019-06-04 17:19:12
 * \author Rolando J. Nieves
 */

#ifndef _SWARMIESRL_NEW_BEHAVIOR_NODECONFIGURATION_HH_
#define _SWARMIESRL_NEW_BEHAVIOR_NODECONFIGURATION_HH_

#include <string>
#include <ostream>
#include <boost/program_options.hpp>


class NodeConfiguration
{
private:
    boost::program_options::options_description m_basicOptions;
    boost::program_options::options_description m_hiddenOptions;
    std::string m_robotName;
    bool m_helpRequested;

public:
    NodeConfiguration();

    virtual ~NodeConfiguration();

    void addCommandLineConfig(int argc, char const **argv);

    inline std::string const& robotName() const { return m_robotName; }

    inline bool helpRequested() const { return m_helpRequested; }

    void printCommandLineHelpTo(std::ostream& stream);
};

#endif /* !_SWARMIESRL_NEW_BEHAVIOR_NODECONFIGURATION_HH_ */

// vim: set ts=4 sw=4 expandtab:
