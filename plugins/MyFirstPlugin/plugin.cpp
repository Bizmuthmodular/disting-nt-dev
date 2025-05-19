#include "api.h"
#include <string.h>
#include <new>  // for placement new

// Your plugin class
struct MyFirstPlugin : _NT_algorithm {
    void step(float* busFrames, int numFramesBy4) {
        int frames = numFramesBy4 * 4;
        float* in = &busFrames[0 * frames];
        float* out = &busFrames[1 * frames];
        for (int i = 0; i < frames; ++i) {
            out[i] = in[i];  // passthrough
        }
    }
};

// Construct a new instance
static _NT_algorithm* construct(const _NT_algorithmMemoryPtrs& ptrs,
                                const _NT_algorithmRequirements& req,
                                const int32_t* specifications) {
    void* mem = ptrs.sram;
    MyFirstPlugin* plugin = new (mem) MyFirstPlugin();
    memset(plugin, 0, sizeof(MyFirstPlugin));
    return plugin;
}

// Declare algorithm memory needs
static void calculateRequirements(_NT_algorithmRequirements& req,
                                  const int32_t* specifications) {
    req.numParameters = 0;
    req.sram = sizeof(MyFirstPlugin);
    req.dram = 0;
    req.dtc = 0;
    req.itc = 0;
}

// Move factory to global scope (avoids __cxa_guard_acquire)
extern "C" {

static const _NT_factory myFirstPluginFactory = {
    .guid = NT_MULTICHAR('D','E','M','O'),
    .name = "My First Plugin",
    .description = "Simple passthrough plugin",
    .numSpecifications = 0,
    .specifications = nullptr,
    .calculateStaticRequirements = nullptr,
    .initialise = nullptr,
    .calculateRequirements = calculateRequirements,
    .construct = construct,
    .parameterChanged = nullptr,
    .step = [](struct _NT_algorithm* self, float* f, int n) {
        static_cast<MyFirstPlugin*>(self)->step(f, n);
    },
    .draw = nullptr,
    .midiRealtime = nullptr,
    .midiMessage = nullptr,
    .tags = 0,
    .hasCustomUi = nullptr,
    .customUi = nullptr,
    .setupUi = nullptr
};

uintptr_t pluginEntry(_NT_selector selector, uint32_t data) {
    switch (selector) {
        case kNT_selector_version: return kNT_apiVersionCurrent;
        case kNT_selector_numFactories: return 1;
        case kNT_selector_factoryInfo: return (uintptr_t)&myFirstPluginFactory;
        default: return 0;
    }
}

}
