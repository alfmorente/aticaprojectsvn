FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_FrontCamera/msg"
  "src/CITIUS_Control_FrontCamera/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/CITIUS_Control_FrontCamera/msg/__init__.py"
  "src/CITIUS_Control_FrontCamera/msg/_msg_frontCameraInfo.py"
  "src/CITIUS_Control_FrontCamera/msg/_msg_ctrlFrontCamera.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
