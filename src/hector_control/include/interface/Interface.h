#pragma once
#include "../messages/LowLevelCmd.h"
#include "../messages/LowlevelState.h"

class Interface {
public:
    virtual ~Interface() = default;
    // virtual void sendRecv(const LowlevelCmd* cmd, LowlevelState* state) = 0;
};
