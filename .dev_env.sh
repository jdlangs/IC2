#!/usr/bin/env bash
# This file exists so I can launch my editor with all the files at once
# since I move around to a lot of different computers. I am pretty much
# the only person working on this project for now so I'm going to leave 
# this here and remove it when I'm done.

if [ "$1" = "all" ]; then
  subl industrial_calibration_libs/CMakeLists.txt
  subl industrial_calibration_libs/package.xml
  subl industrial_calibration_libs/src/*.cpp
  subl industrial_calibration_libs/include/industrial_calibration_libs/*.h
  subl industrial_calibration_libs/test/*.cpp
  subl industrial_calibration_libs/test/*.h
elif [ "$1" = "code" ]; then
  subl industrial_calibration_libs/src/*.cpp
  subl industrial_calibration_libs/include/industrial_calibration_libs/*.h
elif [ "$1" = "config" ]; then
  subl industrial_calibration_libs/CMakeLists.txt
  subl industrial_calibration_libs/package.xml
elif [ "$1" = "tests" ]; then
  subl industrial_calibration_libs/test/*.cpp
  subl industrial_calibration_libs/test/*.h
elif [ "$1" = "targets" ]; then
  subl target_generator/*.sh
  subl target_generator/*.md
  subl target_generator/scripts/*.py
else
  echo "Command: ./dev_env.sh <all> <code> <config> <tests> <targets>"
fi
