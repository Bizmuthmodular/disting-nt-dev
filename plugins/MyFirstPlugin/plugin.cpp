#include "api/distingnt/api.h"
#include <stdint.h>
#include <stdlib.h>

#define MAX_SEQS 16
#define MAX_STEPS 16

enum {
    IDX_RANDOMIZE,
    IDX_INCLUDE_BASE = 1,
    IDX_MIDI_OUT = IDX_INCLUDE_BASE + MAX_SEQS,
    IDX_BPM,
    IDX_CLOCK_BUS,
    IDX_PARAM_BASE
};

enum DirMode { FWD, BWD, RND };

struct Sequence {
    int steps;
    int div;
    int range;
    DirMode dir;
    int pos;
    int divCounter;
    int data[MAX_STEPS];
};

struct Plugin : _NT_algorithm {
    Sequence seqs[MAX_SEQS];
    bool includes[MAX_SEQS];
    bool randomise;
    int clockCounter;

    void step(float* busFrames, int numFramesBy4) {
        int bpm = v[IDX_BPM];
        float freq = bpm / 60.0f * 16.0f;
        int interval = (int)(NT_globals.sampleRate / freq / 4);

        clockCounter++;
        if (clockCounter >= interval) {
            clockCounter = 0;

            for (int ch = 0; ch < MAX_SEQS; ++ch) {
                Sequence& s = seqs[ch];
                if (++s.divCounter >= s.div) {
                    s.divCounter = 0;

                    int idx = s.pos;
                    if (randomise && includes[ch]) {
                        s.data[idx] = rand() % (s.range + 1);
                    }

                    int note = s.data[idx];
                    NT_sendMidi3ByteMessage(
                        v[IDX_MIDI_OUT] == 0 ? kNT_destinationUSB : kNT_destinationBreakout,
                        0x90 | ch, note, 127);

                    if (s.dir == FWD) s.pos = (s.pos + 1) % s.steps;
                    else if (s.dir == BWD) s.pos = (s.pos + s.steps - 1) % s.steps;
                    else if (s.dir == RND) s.pos = rand() % s.steps;
                }
            }

            int bus = v[IDX_CLOCK_BUS] - 1;
            if (bus >= 0 && bus < 28) {
                busFrames[bus * NT_globals.maxFramesPerStep] = 1.0f;
            }
        }
    }
};

static const char* dirLabels[] = { "FWD", "BWD", "RND" };

static _NT_parameter parameters[80];
static _NT_parameterPage pages[4];
static uint8_t paramIndices[80];

static void buildParams() {
    parameters[IDX_RANDOMIZE] = { "Randomise!", 0, 1, 0, kNT_typeBoolean, 0, nullptr };

    for (int i = 0; i < MAX_SEQS; ++i) {
        parameters[IDX_INCLUDE_BASE + i] = { ("Include " + std::to_string(i + 1)).c_str(), 0, 1, 1, kNT_typeBoolean, 0, nullptr };
    }

    parameters[IDX_MIDI_OUT] = { "MIDI Out", 0, 1, 0, kNT_unitEnum, 0, nullptr };
    parameters[IDX_BPM] = { "BPM", 0, 400, 120, kNT_unitBPM, 0, nullptr };
    parameters[IDX_CLOCK_BUS] = { "Clock Output", 1, 28, 1, kNT_unitAudioOutput, 0, nullptr };

    for (int i = 0; i < MAX_SEQS; ++i) {
        int base = IDX_PARAM_BASE + i * 4;
        parameters[base] = { ("Steps " + std::to_string(i + 1)).c_str(), 1, 16, 16, kNT_unitNone, 0, nullptr };
        parameters[base + 1] = { ("Div " + std::to_string(i + 1)).c_str(), 1, 32, 1, kNT_unitNone, 0, nullptr };
        parameters[base + 2] = { ("Range " + std::to_string(i + 1)).c_str(), 0, 127, 127, kNT_unitMIDINote, 0, nullptr };
        parameters[base + 3] = { ("Dir " + std::to_string(i + 1)).c_str(), 0, 2, 0, kNT_unitEnum, 0, dirLabels };
    }

    for (int i = 0; i < 80; ++i) {
        paramIndices[i] = i;
    }

    pages[0] = { "RAND", 17, paramIndices };
    pages[1] = { "MIDI out", 1, &paramIndices[IDX_MIDI_OUT] };
    pages[2] = { "CLOCK", 2, &paramIndices[IDX_BPM] };
    pages[3] = { "PARAM", 60, &paramIndices[IDX_PARAM_BASE] };
}

static void parameterChanged(_NT_algorithm* algo, int p) {
    Plugin* self = static_cast<Plugin*>(algo);

    if (p == IDX_RANDOMIZE) {
        self->randomise = algo->v[p];
    } else if (p >= IDX_INCLUDE_BASE && p < IDX_INCLUDE_BASE + MAX_SEQS) {
        self->includes[p - IDX_INCLUDE_BASE] = algo->v[p];
    } else if (p >= IDX_PARAM_BASE) {
        int seq = (p - IDX_PARAM_BASE) / 4;
        int field = (p - IDX_PARAM_BASE) % 4;
        Sequence& s = self->seqs[seq];
        switch (field) {
            case 0: s.steps = algo->v[p]; break;
            case 1: s.div = algo->v[p]; break;
            case 2: s.range = algo->v[p]; break;
            case 3: s.dir = static_cast<DirMode>(algo->v[p]); break;
        }
    }
}

static void calculateRequirements(_NT_algorithmRequirements& r, const int32_t*) {
    r.numParameters = 80;
    r.sram = sizeof(Plugin);
}

static _NT_algorithm* construct(const _NT_algorithmMemoryPtrs& ptrs, const _NT_algorithmRequirements&, const int32_t*) {
    Plugin* self = new(ptrs.sram) Plugin;
    self->parameters = parameters;
    static _NT_parameterPages allPages = { 4, pages };
    self->parameterPages = &allPages;
    self->v = self->vIncludingCommon + NT_parameterOffset();
    self->clockCounter = 0;
    self->randomise = false;

    for (int i = 0; i < MAX_SEQS; ++i) {
        Sequence& s = self->seqs[i];
        s.steps = 16;
        s.div = 1;
        s.range = 127;
        s.dir = FWD;
        s.pos = 0;
        s.divCounter = 0;
        for (int j = 0; j < MAX_STEPS; ++j) s.data[j] = rand() % 128;
        self->includes[i] = true;
    }

    return self;
}

static const _NT_factory factory = {
    NT_MULTICHAR('M', 'S', 'Q', 'R'),
    "MIDI Pattern Generator",
    "Generates 16 random MIDI sequences",
    0,
    nullptr,
    nullptr,
    nullptr,
    calculateRequirements,
    construct,
    parameterChanged,
    [](auto* self, auto* f, auto n) { static_cast<Plugin*>(self)->step(f, n); },
    nullptr, nullptr, nullptr,
    kNT_tagUtility,
    nullptr, nullptr, nullptr
};

extern "C" uintptr_t pluginEntry(_NT_selector selector, uint32_t data) {
    buildParams();
    switch (selector) {
        case kNT_selector_version: return kNT_apiVersionCurrent;
        case kNT_selector_numFactories: return 1;
        case kNT_selector_factoryInfo: return (uintptr_t)&factory;
        default: return 0;
    }
}
