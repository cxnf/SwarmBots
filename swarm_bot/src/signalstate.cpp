#include "signalstate.h++"

// ----------------- Constructors ------------------------------------------------------------------
SignalState::SignalState()
{
}

// ----------------- Destructors -------------------------------------------------------------------
SignalState::~SignalState()
{
}

// ----------------- Methods -----------------------------------------------------------------------
virtual int SignalState::UpdateState(SwarmBot *bot, FState *state)
{
  switch (this->substate)
    {
    case SS_SEND:
      {
      }
      break;

    case SS_RESTORE:
      {
      }
      break;

    case SS_FINALIZE:
      {
      }
      break;
    }
  return OK_SUCCESS;
}
