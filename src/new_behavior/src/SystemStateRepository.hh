/**
 * \file SystemStateRepository.hh
 * \brief Contains the definition of the \c SystemStateRepository class.
 * \date 2019-06-21 13:18:38
 * \author Rolando J. Nieves
 */

#ifndef _SWARMIESRL_NEW_BEHAVIOR_SYSTEMSTATEREPOSITORY_H_
#define _SWARMIESRL_NEW_BEHAVIOR_SYSTEMSTATEREPOSITORY_H_

#include <utility>
#include <algorithm>
#include <unordered_set>
#include <limits>

#include "SystemStateSample.hh"


namespace std
{

template< typename CoordElem >
struct hash< tuple< CoordElem, CoordElem > >
{
    typedef tuple< CoordElem, CoordElem > argument_type;
    typedef size_t result_type;

    result_type operator() (argument_type const& val) const noexcept
    {
        return hash< CoordElem >{}(get< 0 >(val)) + hash< CoordElem >{}(get< 1 >(val));
    }
};

} // end namespace std

template< unsigned RoverCount >
class SystemStateRepository
{
public:
    using Sample = SystemStateSample< RoverCount >;

private:
    using CoordKey = std::tuple< int, int >;
    using PresenceSet = std::unordered_set< CoordKey >;

    double m_quantizationScale;
    unsigned int m_totalBlockCount;
    Sample m_currentState;
    PresenceSet m_blockPresenceSet;

    void reevalBlockCounts()
    {
        unsigned int roverIdx = 0u;
        unsigned int inTransitCalc = 0u;

        while (roverIdx < RoverCount)
        {
            inTransitCalc += (m_currentState.carrying(roverIdx) ? 1u : 0u);
            roverIdx++;
        }
        m_currentState.setInTransit(inTransitCalc);

        m_currentState.setLocated(
            std::min(
                static_cast< unsigned int >(m_blockPresenceSet.size()),
                m_totalBlockCount
            )
        );

        int atLargeCalc = m_totalBlockCount;
        atLargeCalc -= m_blockPresenceSet.size();
        atLargeCalc -= m_currentState.inTransit();
        atLargeCalc -= m_currentState.collected();

        m_currentState.setAtLarge(
            static_cast< unsigned int >(
                std::max(
                    atLargeCalc,
                    0
                )
            )
        );

    }

public:
    SystemStateRepository(double quantizationScale, unsigned int totalBlockCount):
        m_quantizationScale(quantizationScale),
        m_totalBlockCount(totalBlockCount)
    {
        this->reevalBlockCounts();
    }

    virtual ~SystemStateRepository()
    {}

    inline Sample const& currentState() const
    { return m_currentState; }

    void foundBlockAt(double x, double y)
    {
        auto coordKey = std::make_tuple(
            static_cast< int >(x / m_quantizationScale),
            static_cast< int >(y / m_quantizationScale)
        );

        if (m_blockPresenceSet.find(coordKey) == m_blockPresenceSet.end())
        {
            m_blockPresenceSet.insert(coordKey);
        }

        this->reevalBlockCounts();
    }

    void roverPickedUpBlockAt(unsigned int roverIdx, double x, double y)
    {
        auto coordKey = std::make_tuple(
            static_cast< int >(x / m_quantizationScale),
            static_cast< int >(y / m_quantizationScale)
        );

        m_blockPresenceSet.erase(coordKey);
        m_currentState.setCarrying(roverIdx, true);

        this->reevalBlockCounts();
    }

    void roverDroppedBlock(unsigned int roverIdx)
    {
        m_currentState.setCarrying(roverIdx, false);

        this->reevalBlockCounts();
    }

    void roverCollectedBlock(unsigned int roverIdx)
    {
        m_currentState.setCarrying(roverIdx, false);
        m_currentState.setCollected(
            std::max(
                m_currentState.collected() + 1u,
                m_totalBlockCount
            )
        );

        this->reevalBlockCounts();
    }

    void roverAtPos(unsigned int roverIdx, double x, double y)
    {
        auto roverCoord = std::make_tuple(
            static_cast< int >(x / m_quantizationScale),
            static_cast< int >(y / m_quantizationScale)
        );

        unsigned shortestDistance = std::numeric_limits< unsigned >::max();
        for (auto aBlock = m_blockPresenceSet.begin(); aBlock != m_blockPresenceSet.end(); ++aBlock)
        {
            unsigned distanceVal = std::abs< int >(
                std::get< 0 >(*aBlock) -
                std::get< 0 >(roverCoord)
            );
            distanceVal += std::abs< int >(
                std::get< 1 >(*aBlock) -
                std::get< 1 >(roverCoord)
            );

            if (distanceVal < shortestDistance)
            {
                shortestDistance = distanceVal;
            }
        }

        m_currentState.setDistance(
            roverIdx,
            (m_blockPresenceSet.empty() ?
                -1.0 :
                static_cast< double >(shortestDistance)
            )
        );
    }
};

#endif /* !_SWARMIESRL_NEW_BEHAVIOR_SYSTEMSTATEREPOSITORY_H_ */

// vim: set ts=4 sw=4 expandtab:
