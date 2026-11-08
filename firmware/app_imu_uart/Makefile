# Out-of-tree Crazyflie app. Place this folder next to (or inside) a checkout of
# crazyflie-firmware and point CRAZYFLIE_BASE at it, e.g.
#   make CRAZYFLIE_BASE=../../crazyflie-firmware -j
#   make CRAZYFLIE_BASE=../../crazyflie-firmware cload   # flash over Crazyradio
CRAZYFLIE_BASE ?= ../../crazyflie-firmware

# app-config enables the app layer (CONFIG_APP_ENABLE=y)
OOT_CONFIG := $(PWD)/app-config

include $(CRAZYFLIE_BASE)/tools/make/oot.mk
