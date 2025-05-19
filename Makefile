NT_API_PATH := api
INCLUDE_PATH := $(NT_API_PATH)

PLUGIN_DIR := plugins/MyFirstPlugin
PLUGIN_SRC := $(PLUGIN_DIR)/plugin.cpp
PLUGIN_OBJ := $(PLUGIN_DIR)/plugin.o

all: $(PLUGIN_OBJ)

clean:
	rm -f $(PLUGIN_OBJ)

$(PLUGIN_OBJ): $(PLUGIN_SRC)
	arm-none-eabi-c++ -std=c++11 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -fno-rtti -fno-exceptions -Os -fPIC -Wall -I$(INCLUDE_PATH) -c -o $@ $^
