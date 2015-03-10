FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/RcvJoy/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/RcvJoy/msg/__init__.py"
  "src/RcvJoy/msg/_msg_command.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
