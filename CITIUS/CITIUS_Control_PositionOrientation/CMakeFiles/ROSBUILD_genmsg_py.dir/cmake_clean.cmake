FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_PositionOrientation/msg"
  "src/CITIUS_Control_PositionOrientation/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/CITIUS_Control_PositionOrientation/msg/__init__.py"
  "src/CITIUS_Control_PositionOrientation/msg/_msg_posOriInfo.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
