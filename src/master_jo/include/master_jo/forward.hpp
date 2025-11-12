#ifndef FORWARD_HPP
#define FORWARD_HPP

#include "player.hpp"

namespace master_jo {

class Forward : public Player
{
public:
    Forward(std::shared_ptr<master_jo::MasterRcko> master);

    void stateInitial();
    void stateReady();
    void stateSet();
    void statePlay();
    void stateFinished();
    bool stop_or_go = false;
};

}


#endif