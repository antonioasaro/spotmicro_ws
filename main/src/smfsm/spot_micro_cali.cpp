#define ANTONIO
#include "spot_micro_cali.h"
#include "spot_micro_idle.h"
#include "spot_micro_motion_cmd.h"
#include "spot_micro_transition_idle.h"
#include "i2cpwm_controller.h"

SpotMicroCaliState::SpotMicroCaliState()
{
  // Construcotr, doesn't need to do anything, for now...
  //std::cout << "SpotMicroCaliState Ctor" << std::endl;
}

SpotMicroCaliState::~SpotMicroCaliState()
{
  //std::cout << "SpotMicroCaliState Dtor" << std::endl;
}

void SpotMicroCaliState::handleInputCommands(const smk::BodyState &body_state,
                                             const SpotMicroNodeConfig &smnc,
                                             const Command &cmd,
                                             SpotMicroMotionCmd *smmc,
                                             smk::BodyState *body_state_cmd_)
{
  if (smnc.debug_mode)
  {
    std::cout << "In Spot Micro Cali State" << std::endl;
  }

  // Check if idle command issued, if so, transition to idle state
  if (cmd.getIdleCmd() == true)
  {
    i2cpwm_controller_calibration_disable();
    changeState(smmc, std::make_unique<SpotMicroTransitionIdleState>());
  }
  else
  {
    i2cpwm_controller_calibration_enable();
  }
}
