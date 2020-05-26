#
# Main Makefile. This is basically the same as a component makefile.
#
# This Makefile should, at the very least, just include $(SDK_PATH)/make/component.mk. By default, 
# this will take the sources in the src/ directory, compile them and link them into 
# lib(subdirectory_name).a in the build directory. This behaviour is entirely configurable,
# please read the SDK documents if you need to do this.
#
ULP_APP_NAME ?= ulp-$(COMPONENT_NAME)
ULP_S_SOURCES = $(addprefix $(COMPONENT_PATH)/ulp/, \
	blink.S \
	)
#ULP_EXP_DEP_OBJECTS := main.o ulp-util.o
include $(IDF_PATH)/components/ulp/component_ulp_common.mk
