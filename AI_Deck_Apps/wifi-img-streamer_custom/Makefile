# User Test
#------------------------------------
APP              = wifi-img-streamer
# App sources
APP_SRCS         += wifi-img-streamer.c lib/cpx/src/com.c lib/cpx/src/cpx.c
# App includes
APP_INC	         = lib/cpx/inc
# Compiler flags
APP_CFLAGS 		 += -O3 -g
APP_CFLAGS       += -DconfigUSE_TIMERS=1 -DINCLUDE_xTimerPendFunctionCall=1
# Linker flags
APP_LDFLAGS      =

# Custom linker
APP_LINK_SCRIPT  =

include $(RULES_DIR)/pmsis_rules.mk
