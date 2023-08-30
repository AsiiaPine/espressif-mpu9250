#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := esp-mpu9250

EXTRA_COMPONENT_DIRS := lib/ahrs
						lib/mpu9250

include $(IDF_PATH)/make/project.mk