#Arduino.mk is needed to build this version of repetier! (Although you may be able to get the Arduino UI to work if you try.)

BOARD_TAG    = mega2560
ARDUINO_LIBS =
LDFLAGS += -Wl,-u,vfprintf -lprintf_flt -Wl,--relax

include $(ARDMK_DIR)/Arduino.mk

