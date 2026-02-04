#include "fsm/FSM.h"
#include <unitree/common/thread/thread.hpp>
#include <unistd.h>

int main(int argc, char const *argv[]) {
    if (argc < 2) {
        std::cout << "Usage: ./g1_ctrl [network_interface]" << std::endl;
        return 0;
    }

    FSM fsm(argv[1]);
    fsm.start();

    // 500Hz Loop (2ms)
    unitree::common::ThreadPtr control_thread = unitree::common::CreateRecurrentThreadEx(
        "control_loop", UT_CPU_ID_NONE, 2000, &FSM::update, &fsm
    );

    while (true) { pause(); } // sleep 대신 pause로 효율적 대기
    return 0;
}
