set(COMPONENT_SRCS "main.c")

set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

# # The following lines of boilerplate have to be in your project's CMakeLists
# # in this exact order for cmake to work correctly
# cmake_minimum_required(VERSION 3.5)

# set(EXTRA_COMPONENT_DIRS "lib/ahrs" "lib/mpu9250")
# set(COMPONENTS bootloader esptool_py main ahrs mpu9250)  # This reduces the number of redundant components included

# include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# project(mpu9250)
