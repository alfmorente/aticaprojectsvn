FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_RearCamera/msg"
  "src/CITIUS_Control_RearCamera/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/CITIUS_Control_RearCamera/msg/__init__.py"
  "src/CITIUS_Control_RearCamera/msg/_msg_rearCameraInfo.py"
  "src/CITIUS_Control_RearCamera/msg/_msg_ctrlRearCamera.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
