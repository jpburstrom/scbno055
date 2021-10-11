// PluginSCBNO055.cpp
// Johannes Burström (johannes@ljud.org)

#include "SC_PlugIn.hpp"
#include "SCBNO055.hpp"

static InterfaceTable* ft;

namespace SCBNO055 {

SCBNO055::SCBNO055() {
    mCalcFunc = make_calc_function<SCBNO055, &SCBNO055::next>();
    next(1);
}

void SCBNO055::next(int nSamples) {
    const float* input = in(0);
    const float* gain = in(0);
    float* outbuf = out(0);

    // simple gain function
    for (int i = 0; i < nSamples; ++i) {
        outbuf[i] = input[i] * gain[i];
    }
}

} // namespace SCBNO055

PluginLoad(SCBNO055UGens) {
    // Plugin magic
    ft = inTable;
    registerUnit<SCBNO055::SCBNO055>(ft, "SCBNO055");
}
