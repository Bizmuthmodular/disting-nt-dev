# Set correct path to ARM cross-compiler
CXX := /Applications/ARM/bin/arm-none-eabi-c++
CXXFLAGS := -std=c++11 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb \
            -fno-rtti -fno-exceptions -Os -fPIC -Wall -Iapi

PLUGIN := plugins/sequencer_v1/noculling
SRC := $(PLUGIN).cpp
OBJ := $(PLUGIN).o

all: $(OBJ)

$(OBJ): $(SRC)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ)

