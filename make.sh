#!/bin/bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
if [ -e compile_commands.json ]
then
  echo "sadasda"
else
  ln -s build/compile_commands.json .
fi