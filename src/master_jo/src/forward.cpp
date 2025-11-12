#include "../include/master_jo/forward.hpp"
#include <unistd.h>
namespace master_jo
{

  using namespace std;

  Forward::Forward(std::shared_ptr<master_jo::MasterRcko> master)
      : Player(master)
  {
    cout << endl
         << "HELLO!!" << endl
         << endl;
  }

  void Forward::stateInitial()
  {
    move(false);
  }

  void Forward::stateReady()
  {
    move(false);
  }

  void Forward::stateSet()
  {
    move(false);
  }

  void Forward::statePlay()
  {
    cout << endl
         << "짜잇호" << endl
         << endl;
    if (master->vision.flag == 0)
    {
      stop_or_go = false;
      walkStart(10, 0, 0);
    }
    else if (master->vision.flag == 1)
    {
      if (stop_or_go == false)
      {
        walkStop();
        sleep(1000);
        stop_or_go = true;
      }
      cout << endl
           << "호잇짜" << endl
           << endl;
      walkStart(0, 20, 0);
    }
    else if (master->vision.flag == 2)
    {
      if (stop_or_go == false)
      {
        walkStop();
        sleep(1000);
        stop_or_go = true;
      }
      walkStart(0, -20, 0);
    }
  }

  void Forward::stateFinished()
  {
    move(false);
  }
}