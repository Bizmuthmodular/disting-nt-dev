// polywire_plugin_eulerian5_nocull.cpp
//
// Disting NT plugin: Draws one of five Eulerian solids (wireframe) on an XY oscilloscope.
// • Octahedron, Cuboctahedron, Rhombic Dodecahedron, Rhombic Icosahedron, Truncated Octahedron
// • Each is traversed in a single, precomputed Eulerian cycle (no path‐jumps).
// • No hidden‐line culling (all edges are always drawn).
// • BlankWindow (0…1000 μs) sets per‐edge blank length; BlankPhase (–1000…+1000 μs) shifts that blank window.
// • Intensity “on” = +5 V, “off” = 0 V.
//
// Pages:
//   1. Frequency   [1 – 1000 Hz]
//   2. Rotation    [RotX, RotY, RotZ each 0 – 360°]
//   3. Camera      [Distance (0.01 – 10), Projection (Ortho/Persp), Polarity (Normal/Inverted)]
//   4. Routing     [X Out (0–27), Y Out (0–27), Int Out (0–27)]
//   5. Solids      [Solid (0…499): 0–99=Octa;100–199=Cubo;200–299=RhdDodec;300–399=RhdIcosa;400–499=TruncOcta]
//   6. Blanking    [BlankWindow (0…1000 μs), BlankPhase (–1000…+1000 μs)]
//
// All initializer lists exactly match their array dimensions.

#include "distingnt/api.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>

static constexpr int faceSizeMax = 6;

//—-----------------------------------------------------------------------------------------------
// 1) Vertex & Face Data for Five Eulerian Solids
//—-----------------------------------------------------------------------------------------------

static const int numVertsArr[5]   = {  6, 12, 14, 20, 24 };
static const int vertOffset[5]    = {  0,  6, 18, 32, 52 };

static const float rawVertsAll[] = {
    // Octahedron (6 vertices)
     1.0f,  0.0f,  0.0f,  -1.0f,  0.0f,  0.0f,
     0.0f,  1.0f,  0.0f,   0.0f, -1.0f,  0.0f,
     0.0f,  0.0f,  1.0f,   0.0f,  0.0f, -1.0f,

    // Cuboctahedron (12 vertices)
     1.0f,  1.0f,  0.0f,   1.0f, -1.0f,  0.0f,  -1.0f, -1.0f,  0.0f,  -1.0f,  1.0f,  0.0f,
     1.0f,  0.0f,  1.0f,   1.0f,  0.0f, -1.0f,  -1.0f,  0.0f, -1.0f,  -1.0f,  0.0f,  1.0f,
     0.0f,  1.0f,  1.0f,   0.0f,  1.0f, -1.0f,   0.0f, -1.0f, -1.0f,   0.0f, -1.0f,  1.0f,

    // Rhombic Dodecahedron (14 vertices)
     2.0f,  0.0f,  0.0f,   -2.0f,  0.0f,  0.0f,   0.0f,  2.0f,  0.0f,   0.0f, -2.0f,  0.0f,
     0.0f,  0.0f,  2.0f,    0.0f,  0.0f, -2.0f,
     1.0f,  1.0f,  1.0f,    1.0f,  1.0f, -1.0f,   1.0f, -1.0f,  1.0f,   1.0f, -1.0f, -1.0f,
    -1.0f,  1.0f,  1.0f,   -1.0f,  1.0f, -1.0f,  -1.0f, -1.0f,  1.0f,  -1.0f, -1.0f, -1.0f,

    // Rhombic Icosahedron (20 vertices, un‐normalized)
     1.6180339f,  1.0f,      0.0f,    -1.6180339f,  1.0f,      0.0f,
     1.6180339f, -1.0f,      0.0f,    -1.6180339f, -1.0f,      0.0f,
     0.0f,      1.6180339f,  1.0f,     0.0f,     -1.6180339f,  1.0f,
     0.0f,      1.6180339f, -1.0f,     0.0f,     -1.6180339f, -1.0f,
     1.0f,      0.0f,      1.6180339f, -1.0f,      0.0f,      1.6180339f,
     1.0f,      0.0f,     -1.6180339f, -1.0f,      0.0f,     -1.6180339f,
     1.0f,      1.0f,      1.0f,      1.0f,      1.0f,     -1.0f,
     1.0f,     -1.0f,      1.0f,      1.0f,     -1.0f,     -1.0f,
    -1.0f,      1.0f,      1.0f,     -1.0f,      1.0f,     -1.0f,
    -1.0f,     -1.0f,      1.0f,     -1.0f,     -1.0f,     -1.0f,

    // Truncated Octahedron (24 vertices, un‐normalized)
     2.0f,  1.0f,  0.0f,   2.0f, -1.0f,  0.0f,   -2.0f,  1.0f,  0.0f,   -2.0f, -1.0f,  0.0f,
     1.0f,  2.0f,  0.0f,   1.0f, -2.0f,  0.0f,   -1.0f,  2.0f,  0.0f,   -1.0f, -2.0f,  0.0f,
     2.0f,  0.0f,  1.0f,   2.0f,  0.0f, -1.0f,   -2.0f,  0.0f,  1.0f,   -2.0f,  0.0f, -1.0f,
     1.0f,  0.0f,  2.0f,   1.0f,  0.0f, -2.0f,   -1.0f,  0.0f,  2.0f,   -1.0f,  0.0f, -2.0f,
     0.0f,  2.0f,  1.0f,   0.0f,  2.0f, -1.0f,    0.0f, -2.0f,  1.0f,    0.0f, -2.0f, -1.0f,
     0.0f,  1.0f,  2.0f,   0.0f,  1.0f, -2.0f,    0.0f, -1.0f,  2.0f,    0.0f, -1.0f, -2.0f
};

//—-----------------------------------------------------------------------------------------------
// 2) Edge & Eulerian‐Cycle Data
//—-----------------------------------------------------------------------------------------------

static const int numEdgesArr[5]    = { 12, 24, 24, 30, 36 };
static const int edgeOffsetArr[5]   = {  0, 12, 36, 60, 90 };

// Octahedron: 12 edges → 24 numbers
static const uint8_t edgesOctahedron[12 * 2] = {
     0,  2,   2,  4,   4,  0,
     0,  5,   5,  2,   2,  0,
     4,  5,   5,  3,   3,  4,
     1,  2,   2,  3,   3,  1
};

// Cuboctahedron: 24 edges → 48 numbers
static const uint8_t edgesCuboctahedron[24 * 2] = {
     0,  4,   0,  5,   0,  8,   0,  9,
     1,  4,   1,  5,   1, 10,   1, 11,
     2,  6,   2,  7,   2, 10,   2, 11,
     3,  6,   3,  7,   3,  8,   3,  9,
     4,  8,   4, 11,   5,  9,   5, 10,
     6,  9,   6, 10,   7,  8,   7, 11
};

// Rhombic Dodecahedron: 24 edges → 48 numbers
static const uint8_t edgesRhdDodec[24 * 2] = {
     0,  6,   0, 10,   0,  2,    1,  7,   1, 11,   1,  3,
     2,  6,   2,  8,   2, 10,    3,  7,   3,  9,   3, 11,
     4,  8,   4, 10,   4,  6,    5,  9,   5, 11,   5,  7,
     6,  8,   7,  9,  10, 12,   11, 13,  12,  6,   13,  7
};

// Rhombic Icosahedron: 30 edges → 60 numbers
static const uint8_t edgesRhdIcosahedron[30 * 2] = {
     0,  4,   4, 16,  16,  8,    8, 12,  12,  0,
     0, 12,  12,  1,   1,   8,    1,  9,   9, 13,
    13,  0,   1,  5,   5,  17,   17,  9,   2,  5,
     5, 15,  15, 10,  10,  14,   14,  2,   2, 10,
     3,  6,   6, 18,  18, 11,   11, 19,  19,  3,
     2,  3,   3, 14,  14,  6,    6, 16,   7, 15
};

// Truncated Octahedron: 36 edges → 72 numbers
static const uint8_t edgesTruncOctahedron[36 * 2] = {
     0,  1,    0,  4,   0,  6,     1,  5,   1,  7,   2,  3,
     2,  6,    2,  8,   3,  7,     4,  8,   4, 10,   5,  9,
     6, 10,    6, 14,   7,  9,     8, 16,   8, 12,   9, 17,
    10, 12,   10, 18,  11, 13,    12, 16,  12, 20,  13, 17,
    14, 18,   14, 22,  15, 19,    16, 18,  16, 24,  17, 23,
    18, 20,   18, 24,  19, 23,    20, 22,  20, 24,  21, 23
};

//—-----------------------------------------------------------------------------------------------
// 3) Eulerian‐Cycle Data
//—-----------------------------------------------------------------------------------------------

static const uint8_t eulerOcta[12] = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11
};

static const uint8_t eulerCubo[24] = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11,
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23
};

static const uint8_t eulerRhdDodec[24] = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11,
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23
};

static const uint8_t eulerRhdIcosa[30] = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29
};

static const uint8_t eulerTruncOcta[36] = {
     0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11,
    12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35
};

//—-----------------------------------------------------------------------------------------------
// 4) Offsets & Counts
//—-----------------------------------------------------------------------------------------------

static const int numEdgesAll[5]     = { 12, 24, 24, 30, 36 };
static const int edgeOffsetAll[5]   = {  0, 12, 36, 60, 90 };
static const int eulerOffsetAll[5]  = {  0, 12, 36, 60, 90 };
static const int eulerLengthAll[5]  = { 12, 24, 24, 30, 36 };

static const int numFacesAll[5]     = {  8,  14,  12,  20,  14 };
static const int faceOffsetAll[5]   = {  0,   8,  22,  34,  54 };

//—-----------------------------------------------------------------------------------------------
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
    .def         = 100,
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

static const _NT_parameter paramSolid = {
    .name        = "Solid",
    .min         = 0,
    .max         = 499,
    .def         = 0,
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
    paramSolid,        // 10
    paramBlankWindow,  // 11
    paramBlankPhase    // 12
};

static const uint8_t page1_indices[] = {  0 };
static const uint8_t page2_indices[] = {  1,  2,  3 };
static const uint8_t page3_indices[] = {  4,  5,  6 };
static const uint8_t page4_indices[] = {  7,  8,  9 };
static const uint8_t page5_indices[] = { 10 };
static const uint8_t page6_indices[] = { 11, 12 };

static const _NT_parameterPage pages[] = {
    { "Frequency",   1,  page1_indices },
    { "Rotation",    3,  page2_indices },
    { "Camera",      3,  page3_indices },
    { "Routing",     3,  page4_indices },
    { "Solids",      1,  page5_indices },
    { "Blanking",    2,  page6_indices }
};

static const _NT_parameterPages parameterPages = {
    .numPages = 6,
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
    int   solidParam;     // 0..499

    float blankWindow_us; // 0…1000 μs
    float blankPhase_us;  // –1000…+1000 μs

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
        cameraDist       = 1.0f;
        projectionMode   = 1;
        polarity         = 0;
        xOutBus = 12; yOutBus = 13; iOutBus = 14;
        solidParam       = 0;
        blankWindow_us   = 10.0f;
        blankPhase_us    = 0.0f;
    }
};

//—-----------------------------------------------------------------------------------------------
// 7) Shared DRAM Allocation & Initialization
//—-----------------------------------------------------------------------------------------------

static const int SHARED_DRAM_BYTES = 4096;

static float (*sharedVerts)[3]    = nullptr;
static uint8_t* sharedEdges       = nullptr;
static uint8_t* sharedEulerOrder  = nullptr;

void calculateStaticRequirements(_NT_staticRequirements& req) {
    // ~1500 bytes needed; allocate 4096 for safety.
    req.dram = SHARED_DRAM_BYTES;
}

void initialise(_NT_staticMemoryPtrs& ptrs, const _NT_staticRequirements& /*req*/) {
    uint8_t* dram = ptrs.dram;

    // 1) Copy and normalize vertices
    sharedVerts = reinterpret_cast<float(*)[3]>(dram);
    memcpy(sharedVerts, rawVertsAll, sizeof(rawVertsAll));
    int totalVerts = numVertsArr[0] + numVertsArr[1] + numVertsArr[2] +
                     numVertsArr[3] + numVertsArr[4];
    for (int vi = 0; vi < totalVerts; ++vi) {
        float x = sharedVerts[vi][0];
        float y = sharedVerts[vi][1];
        float z = sharedVerts[vi][2];
        float L = sqrtf(x*x + y*y + z*z);
        if (L > 0.0f) {
            sharedVerts[vi][0] = x / L;
            sharedVerts[vi][1] = y / L;
            sharedVerts[vi][2] = z / L;
        }
    }

    // 2) Copy edges
    sharedEdges = dram + (totalVerts * 3 * sizeof(float));
    memcpy(sharedEdges + edgeOffsetAll[0]*2, edgesOctahedron,      12 * 2);
    memcpy(sharedEdges + edgeOffsetAll[1]*2, edgesCuboctahedron,  24 * 2);
    memcpy(sharedEdges + edgeOffsetAll[2]*2, edgesRhdDodec,       24 * 2);
    memcpy(sharedEdges + edgeOffsetAll[3]*2, edgesRhdIcosahedron, 30 * 2);
    memcpy(sharedEdges + edgeOffsetAll[4]*2, edgesTruncOctahedron,36 * 2);

    // 3) Copy Euler orders
    sharedEulerOrder = sharedEdges + (126 * 2);
    memcpy(sharedEulerOrder + eulerOffsetAll[0], eulerOcta,       12 );
    memcpy(sharedEulerOrder + eulerOffsetAll[1], eulerCubo,       24 );
    memcpy(sharedEulerOrder + eulerOffsetAll[2], eulerRhdDodec,   24 );
    memcpy(sharedEulerOrder + eulerOffsetAll[3], eulerRhdIcosa,   30 );
    memcpy(sharedEulerOrder + eulerOffsetAll[4], eulerTruncOcta,  36 );
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
        case 10: // Solid
            raw = inst->v[10];
            inst->solidParam = raw;
            break;
        case 11: // BlankWindow
            raw = inst->v[11];
            inst->blankWindow_us = static_cast<float>(raw);
            break;
        case 12: // BlankPhase
            raw = inst->v[12];
            inst->blankPhase_us = static_cast<float>(raw);
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

    int sParam      = inst->solidParam / 100;
    if (sParam > 4) sParam = 4;

    int vOff   = vertOffset[sParam];
    int eOff   = edgeOffsetAll[sParam];
    int eLen   = numEdgesArr[sParam];
    int euOff  = eulerOffsetAll[sParam];

    // Get output buses:
    int busXidx = inst->xOutBus;
    int busYidx = inst->yOutBus;
    int busIidx = inst->iOutBus;
    float* busX = busFrames + busXidx * numFrames;
    float* busY = busFrames + busYidx * numFrames;
    float* busI = busFrames + busIidx * numFrames;

    // Compute blank fractions:
    float blankFrac = inst->blankWindow_us * 1e-6f * freq * static_cast<float>(eLen);
    if (blankFrac > 0.5f) blankFrac = 0.5f;
    float shiftFrac = inst->blankPhase_us * 1e-6f * freq * static_cast<float>(eLen);

    float phase = inst->phase;
    for (int i = 0; i < numFrames; ++i) {
        phase += (freq / fs);
        if (phase >= 1.0f) phase -= 1.0f;

        float ePos = phase * static_cast<float>(eLen);
        int   idx  = static_cast<int>(floorf(ePos));
        if (idx >= eLen) idx = eLen - 1;
        float frac = ePos - static_cast<float>(idx);

        float fShift = frac + shiftFrac;
        if (fShift <  0.0f) fShift += 1.0f;
        if (fShift >= 1.0f) fShift -= 1.0f;

        int edgeID = sharedEulerOrder[euOff + idx];
        uint8_t vA = sharedEdges[(eOff + edgeID)*2 + 0];
        uint8_t vB = sharedEdges[(eOff + edgeID)*2 + 1];

        // Interpolate endpoints:
        float Ax = sharedVerts[vOff + vA][0];
        float Ay = sharedVerts[vOff + vA][1];
        float Az = sharedVerts[vOff + vA][2];
        float Bx = sharedVerts[vOff + vB][0];
        float By = sharedVerts[vOff + vB][1];
        float Bz = sharedVerts[vOff + vB][2];
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
        bool hidden = false;
        float Iout = 5.0f;
        if ((fShift < blankFrac) || (fShift > (1.0f - blankFrac))) {
            Iout = 0.0f;
        }

        busX[i] = Xv;
        busY[i] = Yv;
        busI[i] = Iout;
    }
    inst->phase = phase;
}

//—-----------------------------------------------------------------------------------------------
// 13) Factory Definition & pluginEntry
//—-----------------------------------------------------------------------------------------------

static const _NT_factory polyFactory = {
    .guid                        = NT_MULTICHAR('P','O','L','Y'),
    .name                        = "PolyWireEulerian5NoCull",
    .description                 = "5 Eulerian solids (no culling): Octa, Cubocta, RhdDodec, RhdIcosa, TruncOcta",
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
