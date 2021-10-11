// PluginSCBNO055.hpp
// Johannes Burström (johannes@ljud.org)

#pragma once

#include "SC_PlugIn.hpp"

namespace SCBNO055 {

class SCBNO055 : public SCUnit {
public:
    SCBNO055();

    // Destructor
    // ~SCBNO055();

private:
    // Calc function
    void next(int nSamples);

    // Member variables
};

} // namespace SCBNO055
