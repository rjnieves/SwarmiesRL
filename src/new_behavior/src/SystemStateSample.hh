/**
 * \file SystemStateSample.hh
 * \brief Contains the definition of the \c SystemStateSample class.
 * \date 2019-06-20 16:13:27
 * \author Rolando J. Nieves
 */

#ifndef _SWARMIESRL_NEW_BEHAVIOR_SYSTEMSTATESAMPLE_HH_
#define _SWARMIESRL_NEW_BEHAVIOR_SYSTEMSTATESAMPLE_HH_

#include <array>
#include <utility>
#include <ostream>
#include <sstream>
#include <stdexcept>


template< unsigned RoverCount >
class SystemStateSample
{
private:
    static constexpr unsigned int AT_LARGE_IDX = 0u;
    static constexpr unsigned int LOCATED_IDX = 1u;
    static constexpr unsigned int IN_TRANSIT_IDX = 2u;
    static constexpr unsigned int COLLECTED_IDX = 3u;
    static constexpr unsigned int CARRYING_START_IDX = 4u;
    static constexpr unsigned int DISTANCE_START_IDX = CARRYING_START_IDX + RoverCount;
    static constexpr unsigned int STATE_VECTOR_SIZE = DISTANCE_START_IDX + RoverCount;

    using StateStorage = std::array< double, STATE_VECTOR_SIZE >;

    StateStorage m_storage;

    void validateRoverIndex(unsigned int roverIdx)
    {
        if (roverIdx >= RoverCount)
        {
            std::stringstream errMsg;
            errMsg << "Rover index (" << roverIdx << ") out of range.";
            throw std::runtime_error(errMsg.str());
        }        
    }

public:
    SystemStateSample()
    {
        m_storage.fill(0.0);
        unsigned int distIdx = DISTANCE_START_IDX;
        while (distIdx < STATE_VECTOR_SIZE)
        {
            m_storage[distIdx] = -1.0;
            distIdx++;
        }
    }

    SystemStateSample(SystemStateSample const& other):
        m_storage(other.m_storage)
    {}

    SystemStateSample(SystemStateSample&& other):
        m_storage(std::move(other.m_storage))
    {}

    typename StateStorage::const_iterator begin() const
    { return m_storage.begin(); }

    typename StateStorage::const_iterator end() const
    { return m_storage.end(); }

    std::size_t size() const
    { return STATE_VECTOR_SIZE; }

    inline unsigned int atLarge() const
    { return static_cast< unsigned int >(m_storage[AT_LARGE_IDX]); }

    inline void setAtLarge(unsigned int count)
    { m_storage[AT_LARGE_IDX] = static_cast< double >(count); }

    inline unsigned int located() const
    { return static_cast< unsigned int >(m_storage[LOCATED_IDX]); }

    inline void setLocated(unsigned int count)
    { m_storage[LOCATED_IDX] = static_cast< double >(count); }

    inline unsigned int inTransit() const
    { return static_cast< unsigned int >(m_storage[IN_TRANSIT_IDX]); }

    inline void setInTransit(unsigned int count)
    { m_storage[IN_TRANSIT_IDX] = static_cast< double >(count); }

    inline unsigned int collected() const
    { return static_cast< unsigned int >(m_storage[COLLECTED_IDX]); }

    inline void setCollected(unsigned int count)
    { m_storage[COLLECTED_IDX] = static_cast< double >(count); }

    bool carrying(unsigned int roverIdx) const
    {
        this->validateRoverIndex(roverIdx);
        return m_storage[CARRYING_START_IDX + roverIdx] == 1.0;
    }

    void setCarrying(unsigned int roverIdx, bool isCarrying)
    {
        this->validateRoverIndex(roverIdx);
        m_storage[CARRYING_START_IDX + roverIdx] = (isCarrying ? 1.0 : 0.0);
    }

    double distance(unsigned int roverIdx) const
    {
        this->validateRoverIndex(roverIdx);
        return m_storage[DISTANCE_START_IDX + roverIdx];
    }

    void setDistance(unsigned int roverIdx, double theDistance)
    {
        this->validateRoverIndex(roverIdx);
        m_storage[DISTANCE_START_IDX + roverIdx] = theDistance;
    }

    SystemStateSample& operator= (SystemStateSample const& other)
    { m_storage = other.m_storage; return *this; }

    SystemStateSample& operator= (SystemStateSample&& other)
    { m_storage = std::move(other.m_storage); return *this; }

    friend std::ostream& operator<< (std::ostream& os, SystemStateSample const& theState)
    {
        os << "[ ";
        unsigned int idx = 0u;
        while (idx < STATE_VECTOR_SIZE)
        {
            if (idx >= DISTANCE_START_IDX)
            {
                os << std::fixed << theState.m_storage[idx] << " ";
            }
            else if (idx >= CARRYING_START_IDX)
            {
                os << (theState.m_storage[idx] == 1.0 ? "true" : "false") << " ";
            }
            else
            {
                os << static_cast< int >(theState.m_storage[idx]) << " ";
            }

            idx++;
        }

        os << "]";

        return os;
    }
};

#endif /* !_SWARMIESRL_NEW_BEHAVIOR_SYSTEMSTATESAMPLE_HH_ */

// vim: set ts=4 sw=4 expandtab:
