#include "../include/master_jo/forward.hpp"

namespace master_jo {

using namespace std;

Forward::Forward(std::shared_ptr<master_jo::MasterRcko> master)
    : Player(master) {
  cout << endl << "HELLO!!" << endl << endl;
}

void Forward::stateInitial() {
  move(false);
}

void Forward::stateReady() { 
  move(false);
}

void Forward::stateSet() {
  move(false);
}

void Forward::statePlay() {
  walkStart(10, 0, 0);
}

void Forward::stateFinished() {
  move(false);
}
}