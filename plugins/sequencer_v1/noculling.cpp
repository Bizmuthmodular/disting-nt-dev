// cube_wireframe_nocull.cpp
//
// Disting NT plugin: Draws a rotating wireframe cube on an XY oscilloscope.
// • The cube is normalised using a single scale factor so all vertices lie on a
//   unit sphere.
// • Each edge is traversed in a fixed order with blanked reposition moves so the
//   geometry is correct with no undesired path jumps.
// • No hidden‐line culling (all edges are always drawn when the beam is on).
// • BlankWindow (0…1000 μs) sets per‐edge blank length; BlankPhase (–1000…+1000 μs) shifts that blank window.
// • Intensity “on” = +5 V, “off” = 0 V.
//
// Pages:
//   1. Frequency   [1 – 1000 Hz]
//   2. Rotation    [RotX, RotY, RotZ each 0 – 360°]
//   3. Camera      [Distance (0.01 – 10), Projection (Ortho/Persp), Polarity (Normal/Inverted)]
//   4. Routing     [X Out (0–27), Y Out (0–27), Int Out (0–27)]
//   5. Blanking    [BlankWindow (0…1000 μs), BlankPhase (–1000…+1000 μs)]
//   6. Quantize    [Resolution (0–100)]
//   7. AmpMod      [AmpMod, AmpCorse, AmpFine, AmpWave, AmpPhase]
//
// All initializer lists exactly match their array dimensions.

#include "distingnt/api.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>

static constexpr int faceSizeMax = 6;

//—-----------------------------------------------------------------------------------------------
// 1) Cube Vertex Data & Drawing Segments

static const float rawCubeVerts[8 * 3] = {
    -1.0f, -1.0f, -1.0f,
     1.0f, -1.0f, -1.0f,
     1.0f,  1.0f, -1.0f,
    -1.0f,  1.0f, -1.0f,
    -1.0f, -1.0f,  1.0f,
     1.0f, -1.0f,  1.0f,
     1.0f,  1.0f,  1.0f,
    -1.0f,  1.0f,  1.0f
};

struct Segment { uint8_t a; uint8_t b; uint8_t draw; };

static const Segment cubeSegments[] = {
    {0,1,1}, {1,2,1}, {2,3,1}, {3,0,1},
    {0,4,1}, {4,5,1}, {5,6,1}, {6,7,1}, {7,4,1},
    {4,1,0}, {1,5,1}, {5,2,0}, {2,6,1}, {6,3,0},
    {3,7,1}, {7,0,0}
};

static const int numSegments = sizeof(cubeSegments)/sizeof(cubeSegments[0]);

// 5) Parameter Definitions
//—-----------------------------------------------------------------------------------------------

static const _NT_parameter paramFreq = {
    .name        = "Frequency",
    .min         = 1,
    .max         = 1000,
    .def         = 50,
    .unit        = kNT_unitHz,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

static const _NT_parameter paramRotX = {
    .name        = "RotX",
    .min         = 0,
    .max         = 360,
    .def         = 0,
    .unit        = kNT_unitNone,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

static const _NT_parameter paramRotY = {
    .name        = "RotY",
    .min         = 0,
    .max         = 360,
    .def         = 0,
    .unit        = kNT_unitNone,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

static const _NT_parameter paramRotZ = {
    .name        = "RotZ",
    .min         = 0,
    .max         = 360,
    .def         = 0,
    .unit        = kNT_unitNone,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

static const _NT_parameter paramDistance = {
    .name        = "Distance",
    .min         = 1,    // scales 0.01…10.00
    .max         = 1000,
    .def         = 500,
    .unit        = kNT_unitNone,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

static const char* const projEnumStrings[] = { "Orthographic", "Perspective", NULL };
static const _NT_parameter paramProjection = {
    .name        = "Projection",
    .min         = 0,
    .max         = 1,
    .def         = 1,
    .unit        = kNT_unitEnum,
    .scaling     = kNT_scalingNone,
    .enumStrings = projEnumStrings
};

static const char* const polarityEnum[] = { "Normal", "Inverted", NULL };
static const _NT_parameter paramPolarity = {
    .name        = "Polarity",
    .min         = 0,
    .max         = 1,
    .def         = 0,
    .unit        = kNT_unitEnum,
    .scaling     = kNT_scalingNone,
    .enumStrings = polarityEnum
};

static const _NT_parameter paramXOut = {
    .name        = "X Out",
    .min         = 0,
    .max         = 27,
    .def         = 12,
    .unit        = kNT_unitNone,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

static const _NT_parameter paramYOut = {
    .name        = "Y Out",
    .min         = 0,
    .max         = 27,
    .def         = 13,
    .unit        = kNT_unitNone,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

static const _NT_parameter paramIOut = {
    .name        = "Int Out",
    .min         = 0,
    .max         = 27,
    .def         = 14,
    .unit        = kNT_unitNone,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};


static const _NT_parameter paramBlankWindow = {
    .name        = "BlankWindow",
    .min         = 0,
    .max         = 1000,
    .def         = 10,
    .unit        = kNT_unitNone, // μs
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

static const _NT_parameter paramBlankPhase = {
    .name        = "BlankPhase",
    .min         = -1000,
    .max         = +1000,
    .def         = 0,
    .unit        = kNT_unitNone, // μs
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

// Frequency Modulation parameters ------------------------------------------------

static const char* const modCourseStrings[] = {
    "/4", "/3", "/2", "0",
    "x2",  "x3",  "x4",  "x5",  "x6",  "x7",  "x8",  "x9",
    "x10", "x11", "x12", "x13", "x14", "x15", "x16", "x17",
    "x18", "x19", "x20", "x21", "x22", "x23", "x24", "x25",
    "x26", "x27", "x28", "x29", "x30", "x31", "x32",
    NULL
};


// Amplitude Modulation parameters ------------------------------------------------
static const _NT_parameter paramResolution = {
    .name        = "Resolution",
    .min         = 0,
    .max         = 100,
    .def         = 0,
    .unit        = kNT_unitNone,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

// Amplitude Modulation parameters ----------------------------------------------

static const _NT_parameter paramAmpMod = {
    .name        = "AmpMod",
    .min         = 0,
    .max         = 127,
    .def         = 0,
    .unit        = kNT_unitNone,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

static const _NT_parameter paramAmpCorse = {
    .name        = "AmpCorse",
    .min         = 0,
    .max         = 34,
    .def         = 4,
    .unit        = kNT_unitEnum,
    .scaling     = kNT_scalingNone,
    .enumStrings = modCourseStrings
};

static const char* const modWaveStrings[] = {
    "Square", "Triangle", "Saw", "Ramp", "Sine", NULL
};

static const _NT_parameter paramAmpFine = {
    .name        = "AmpFine",
    .min         = -100,
    .max         = 100,
    .def         = 0,
    .unit        = kNT_unitNone,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

static const _NT_parameter paramAmpWave = {
    .name        = "AmpWave",
    .min         = 0,
    .max         = 4,
    .def         = 4,
    .unit        = kNT_unitEnum,
    .scaling     = kNT_scalingNone,
    .enumStrings = modWaveStrings
};

static const _NT_parameter paramAmpPhase = {
    .name        = "AmpPhase",
    .min         = 0,
    .max         = 360,
    .def         = 0,
    .unit        = kNT_unitNone,
    .scaling     = kNT_scalingNone,
    .enumStrings = NULL
};

static const _NT_parameter allParams[] = {
    paramFreq,         //  0
    paramRotX,         //  1
    paramRotY,         //  2
    paramRotZ,         //  3
    paramDistance,     //  4
    paramProjection,   //  5
    paramPolarity,     //  6
    paramXOut,         //  7
    paramYOut,         //  8
    paramIOut,         //  9
    paramBlankWindow,  // 10
    paramBlankPhase,   // 11
    paramResolution,   // 12
    paramAmpMod,       // 13
    paramAmpCorse,     // 14
    paramAmpFine,      // 15
    paramAmpWave,      // 16
    paramAmpPhase      // 17
};

static const uint8_t page1_indices[] = { 0 };
static const uint8_t page2_indices[] = { 1, 2, 3 };
static const uint8_t page3_indices[] = { 4, 5, 6 };
static const uint8_t page4_indices[] = { 7, 8, 9 };
static const uint8_t page5_indices[] = { 10, 11 };
static const uint8_t page6_indices[] = { 12 };
static const uint8_t page7_indices[] = { 13, 14, 15, 16, 17 };

static const _NT_parameterPage pages[] = {
    { "Frequency",   1,  page1_indices },
    { "Rotation",    3,  page2_indices },
    { "Camera",      3,  page3_indices },
    { "Routing",     3,  page4_indices },
    { "Blanking",    2,  page5_indices },
    { "Quantize",    1,  page6_indices },
    { "AmpMod",      5,  page7_indices }
};

static const _NT_parameterPages parameterPages = {
    .numPages = 7,
    .pages    = pages
};

//—-----------------------------------------------------------------------------------------------
// 6) Per‐Instance State Structure
//—-----------------------------------------------------------------------------------------------

struct PolyInstance : public _NT_algorithm {
    float phase;
    float sinX, cosX;
    float sinY, cosY;
    float sinZ, cosZ;
    float freq_Hz;
    float cameraDist;
    int   projectionMode; // 0=Ortho, 1=Persp
    int   polarity;       // 0=Normal, 1=Inverted
    int   xOutBus, yOutBus, iOutBus;

    // Amplitude modulation state
    float ampModAmt;       // 0..1
    int   ampCorseIdx;     // 0..34
    int   ampFine;         // -100..100 (0.1 Hz units)
    int   ampWave;         // 0..4
    float ampPhaseOffset;  // 0..1
    float ampPhase;        // 0..1 running phase

    float blankWindow_us; // 0…1000 μs
    float blankPhase_us;  // –1000…+1000 μs
    int   resolution;     // 0..100

    PolyInstance() {
        parameters       = nullptr;
        parameterPages   = nullptr;
        vIncludingCommon = nullptr;
        v                = nullptr;
        phase            = 0.0f;
        sinX = 0.0f; cosX = 1.0f;
        sinY = 0.0f; cosY = 1.0f;
        sinZ = 0.0f; cosZ = 1.0f;
        freq_Hz          = 50.0f;
        cameraDist       = 5.0f;
        projectionMode   = 1;
        polarity         = 0;
        xOutBus = 12; yOutBus = 13; iOutBus = 14;
        resolution       = 0;
        ampModAmt        = 0.0f;
        ampCorseIdx      = 4;
        ampFine          = 0;
        ampWave          = 4;
        ampPhaseOffset   = 0.0f;
        ampPhase         = 0.0f;
        blankWindow_us   = 10.0f;
        blankPhase_us    = 0.0f;
    }
};

//—-----------------------------------------------------------------------------------------------
// 7) Shared DRAM Allocation & Initialization
//—-----------------------------------------------------------------------------------------------

static const int SHARED_DRAM_BYTES = 256;


static float (*sharedVerts)[3] = nullptr;

void calculateStaticRequirements(_NT_staticRequirements& req) {
    req.dram = SHARED_DRAM_BYTES;
}

void initialise(_NT_staticMemoryPtrs& ptrs, const _NT_staticRequirements& /*req*/) {
    uint8_t* dram = ptrs.dram;
    sharedVerts = reinterpret_cast<float(*)[3]>(dram);
    memcpy(sharedVerts, rawCubeVerts, sizeof(rawCubeVerts));

    float maxL = 0.0f;
    for (int i = 0; i < 8; ++i) {
        float x = sharedVerts[i][0];
        float y = sharedVerts[i][1];
        float z = sharedVerts[i][2];
        float L = sqrtf(x*x + y*y + z*z);
        if (L > maxL) maxL = L;
    }
    if (maxL == 0.0f) maxL = 1.0f;
    float invL = 1.0f / maxL;
    for (int i = 0; i < 8; ++i) {
        sharedVerts[i][0] *= invL;
        sharedVerts[i][1] *= invL;
        sharedVerts[i][2] *= invL;
    }
}

//—-----------------------------------------------------------------------------------------------
// 8) Per-Instance Memory Requirements
//—-----------------------------------------------------------------------------------------------

void calculateRequirements(_NT_algorithmRequirements& req, const int32_t* /*specs*/) {
    req.numParameters = sizeof(allParams) / sizeof(allParams[0]);
    req.sram = 4096;
    req.dram = 0;
    req.dtc  = 0;
    req.itc  = 0;
}

//—-----------------------------------------------------------------------------------------------
// 9) Utility: Normalize a 3‐vector
//—-----------------------------------------------------------------------------------------------

static void normalize3(float* v) {
    float L = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (L > 0.0f) {
        v[0] /= L;
        v[1] /= L;
        v[2] /= L;
    }
}

static inline float oscWave(int type, float phase) {
    phase -= floorf(phase);
    switch (type) {
        case 0: // Square
            return (phase < 0.5f) ? 1.0f : -1.0f;
        case 1: // Triangle
            return (phase < 0.5f) ? (4.0f*phase-1.0f) : (3.0f-4.0f*phase);
        case 2: // Saw
            return 1.0f - 2.0f*phase;
        case 3: // Ramp
            return 2.0f*phase - 1.0f;
        case 4: // Sine
        default:
            return sinf(2.0f * 3.14159265f * phase);
    }
}

static inline float getCourseFactor(int idx) {
    if (idx == 0) return 0.25f;
    if (idx == 1) return 1.0f / 3.0f;
    if (idx == 2) return 0.5f;
    if (idx == 3) return 1.0f;
    return static_cast<float>(idx - 2);
}

//—-----------------------------------------------------------------------------------------------
// 10) ParameterChanged
//—-----------------------------------------------------------------------------------------------

void parameterChanged(_NT_algorithm* baseSelf, int p) {
    PolyInstance* inst = reinterpret_cast<PolyInstance*>(baseSelf);
    int16_t raw;
    switch (p) {
        case 0: // Frequency
            raw = inst->v[0];
            inst->freq_Hz = static_cast<float>(raw);
            break;
        case 1: // RotX
            raw = inst->v[1];
            {
                float r = raw * (3.14159265f / 180.0f);
                inst->sinX = sinf(r);
                inst->cosX = cosf(r);
            }
            break;
        case 2: // RotY
            raw = inst->v[2];
            {
                float r = raw * (3.14159265f / 180.0f);
                inst->sinY = sinf(r);
                inst->cosY = cosf(r);
            }
            break;
        case 3: // RotZ
            raw = inst->v[3];
            {
                float r = raw * (3.14159265f / 180.0f);
                inst->sinZ = sinf(r);
                inst->cosZ = cosf(r);
            }
            break;
        case 4: // Distance
            raw = inst->v[4];
            {
                float d = raw * 0.01f;
                if (d < 0.111f) d = 0.111f;
                inst->cameraDist = d;
            }
            break;
        case 5: // Projection
            raw = inst->v[5];
            inst->projectionMode = (raw != 0 ? 1 : 0);
            break;
        case 6: // Polarity
            raw = inst->v[6];
            inst->polarity = raw;
            break;
        case 7: // X Out
            raw = inst->v[7];
            inst->xOutBus = raw;
            break;
        case 8: // Y Out
            raw = inst->v[8];
            inst->yOutBus = raw;
            break;
        case 9: // Int Out
            raw = inst->v[9];
            inst->iOutBus = raw;
            break;
        case 10: // BlankWindow
            raw = inst->v[10];
            inst->blankWindow_us = static_cast<float>(raw);
            break;
        case 11: // BlankPhase
            raw = inst->v[11];
            inst->blankPhase_us = static_cast<float>(raw);
            break;
        case 12: // Resolution
            raw = inst->v[12];
            inst->resolution = raw;
            break;
        case 13: // AmpMod
            raw = inst->v[13];
            inst->ampModAmt = static_cast<float>(raw) / 127.0f;
            break;
        case 14: // AmpCorse
            raw = inst->v[14];
            inst->ampCorseIdx = raw;
            break;
        case 15: // AmpFine
            raw = inst->v[15];
            inst->ampFine = raw;
            break;
        case 16: // AmpWave
            raw = inst->v[16];
            inst->ampWave = raw;
            break;
        case 17: // AmpPhase
            raw = inst->v[17];
            inst->ampPhaseOffset = static_cast<float>(raw) / 360.0f;
            break;
        default:
            break;
    }
}

//—-----------------------------------------------------------------------------------------------
// 11) Construct Algorithm Instance
//—-----------------------------------------------------------------------------------------------

_NT_algorithm* constructAlgorithm(const _NT_algorithmMemoryPtrs& ptrs,
                                  const _NT_algorithmRequirements& /*req*/,
                                  const int32_t* /*specs*/) {
    void* sramBase = ptrs.sram;
    PolyInstance* inst = new (sramBase) PolyInstance();
    inst->parameters       = allParams;
    inst->parameterPages   = &parameterPages;
    return reinterpret_cast<_NT_algorithm*>(inst);
}

//—-----------------------------------------------------------------------------------------------
// 12) Audio‐Rate step: Draw Eulerian cycle (no culling)
//—-----------------------------------------------------------------------------------------------

void step(_NT_algorithm* baseSelf, float* busFrames, int numFramesBy4) {
    PolyInstance* inst = reinterpret_cast<PolyInstance*>(baseSelf);

    int   numFrames = numFramesBy4 * 4;
    float fs        = static_cast<float>(NT_globals.sampleRate);
    float freq      = inst->freq_Hz;
    int   eLen      = numSegments;

    // Get output buses:
    int busXidx = inst->xOutBus;
    int busYidx = inst->yOutBus;
    int busIidx = inst->iOutBus;
    float* busX = busFrames + busXidx * numFrames;
    float* busY = busFrames + busYidx * numFrames;
    float* busI = busFrames + busIidx * numFrames;

    // Compute blank fractions:
    // Use a fixed reference frequency so blanking covers the same path length
    // regardless of the drawing frequency. This keeps reposition moves hidden
    // even when the animation slows down.
    const float freqRef = 50.0f;  // reference = default Frequency parameter
    float blankFrac = inst->blankWindow_us * 1e-6f * freqRef * static_cast<float>(eLen);
    if (blankFrac > 0.5f) blankFrac = 0.5f;
    float shiftFrac = inst->blankPhase_us * 1e-6f * freqRef * static_cast<float>(eLen);

    float ampCourseFac = getCourseFactor(inst->ampCorseIdx);
    float ampFreqBase  = freq * ampCourseFac;
    float ampFreq      = ampFreqBase + (static_cast<float>(inst->ampFine) * 0.1f);
    if (ampFreq < 0.0f) ampFreq = 0.0f;
    float ampPhase     = inst->ampPhase;

    float phase = inst->phase;
    for (int i = 0; i < numFrames; ++i) {
        

        phase += (freq / fs);
        if (phase >= 1.0f) phase -= 1.0f;

        // Amplitude modulation oscillator
        ampPhase += (ampFreq / fs);
        if (ampPhase >= 1.0f) ampPhase -= floorf(ampPhase);
        float ampVal = oscWave(inst->ampWave, ampPhase + inst->ampPhaseOffset);
        float ampMul = 1.0f + inst->ampModAmt * ampVal;

        float ePos = phase * static_cast<float>(eLen);
        int   idx  = static_cast<int>(floorf(ePos));
        if (idx >= eLen) idx = eLen - 1;
        float frac = ePos - static_cast<float>(idx);

        float fShift = frac + shiftFrac;
        if (fShift <  0.0f) fShift += 1.0f;
        if (fShift >= 1.0f) fShift -= 1.0f;

        const Segment& seg = cubeSegments[idx];
        uint8_t vA = seg.a;
        uint8_t vB = seg.b;

        // Interpolate endpoints:
        float Ax = sharedVerts[vA][0];
        float Ay = sharedVerts[vA][1];
        float Az = sharedVerts[vA][2];
        float Bx = sharedVerts[vB][0];
        float By = sharedVerts[vB][1];
        float Bz = sharedVerts[vB][2];
        float Px = (1.0f - frac)*Ax + frac*Bx;
        float Py = (1.0f - frac)*Ay + frac*By;
        float Pz = (1.0f - frac)*Az + frac*Bz;

        // Rotate around X:
        float Y1 = inst->cosX * Py - inst->sinX * Pz;
        float Z1 = inst->sinX * Py + inst->cosX * Pz;
        float X1 = Px;
        // Rotate around Y:
        float X2 = inst->cosY * X1 + inst->sinY * Z1;
        float Z2 = -inst->sinY * X1 + inst->cosY * Z1;
        float Y2 = Y1;
        // Rotate around Z:
        float Xr = inst->cosZ * X2 - inst->sinZ * Y2;
        float Yr = inst->sinZ * X2 + inst->cosZ * Y2;
        float Zr = Z2;

        if (inst->resolution > 0) {
            float res = static_cast<float>(inst->resolution);
            float scaleQ = res * 0.5f;
            Xr = roundf((Xr + 1.0f) * scaleQ) / scaleQ - 1.0f;
            Yr = roundf((Yr + 1.0f) * scaleQ) / scaleQ - 1.0f;
            Zr = roundf((Zr + 1.0f) * scaleQ) / scaleQ - 1.0f;
        }

        // Project:
        float Xv, Yv;
        if (inst->projectionMode == 1) {
            if (inst->polarity == 0) {
                float dcam = Zr + inst->cameraDist;
                if (dcam == 0.0f) dcam = 0.0001f;
                float scale = inst->cameraDist / dcam;
                Xv = 5.0f * (Xr * scale);
                Yv = 5.0f * (Yr * scale);
            } else {
                float dcam = Zr + inst->cameraDist;
                float scale = dcam / inst->cameraDist;
                Xv = 5.0f * (Xr * scale);
                Yv = 5.0f * (Yr * scale);
            }
        } else {
            Xv = 5.0f * Xr;
            Yv = 5.0f * Yr;
        }

        // No culling: always draw
        float Iout = (seg.draw ? 5.0f : 0.0f);
        if (seg.draw && ((fShift < blankFrac) || (fShift > (1.0f - blankFrac)))) {
            Iout = 0.0f;
        }

        float outX = Xv * ampMul;
        float outY = Yv * ampMul;
        busX[i] = outX;
        busY[i] = outY;
        busI[i] = Iout;
    }
    inst->phase     = phase;
    inst->ampPhase  = ampPhase;
}

//—-----------------------------------------------------------------------------------------------
// 13) Factory Definition & pluginEntry
//—-----------------------------------------------------------------------------------------------

static const _NT_factory polyFactory = {
    .guid                        = NT_MULTICHAR('P','O','L','Y'),
    .name                        = "CubeWireNoCull",
    .description                 = "Wireframe cube (no culling)",
    .numSpecifications           = 0,
    .specifications              = nullptr,
    .calculateStaticRequirements = calculateStaticRequirements,
    .initialise                  = initialise,
    .calculateRequirements       = calculateRequirements,
    .construct                   = constructAlgorithm,
    .parameterChanged            = parameterChanged,
    .step                        = step,
    .draw                        = nullptr,
    .midiRealtime                = nullptr,
    .midiMessage                 = nullptr,
    .tags                        = kNT_tagUtility,
    .hasCustomUi                 = nullptr,
    .customUi                    = nullptr,
    .setupUi                     = nullptr
};

extern "C" uintptr_t pluginEntry(_NT_selector selector, uint32_t data) {
    switch (selector) {
        case kNT_selector_version:
            return kNT_apiVersion5;
        case kNT_selector_numFactories:
            return 1;
        case kNT_selector_factoryInfo:
            if (data == 0) return reinterpret_cast<uintptr_t>(&polyFactory);
            return 0;
        default:
            return 0;
    }
}
