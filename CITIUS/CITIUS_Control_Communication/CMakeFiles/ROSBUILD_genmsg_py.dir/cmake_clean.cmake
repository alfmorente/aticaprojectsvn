FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_Communication/msg"
  "src/CITIUS_Control_Communication/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/CITIUS_Control_Communication/msg/__init__.py"
  "src/CITIUS_Control_Communication/msg/_msg_frontCameraInfo.py"
  "src/CITIUS_Control_Communication/msg/_msg_ctrlFrontCamera.py"
  "src/CITIUS_Control_Communication/msg/_msg_posOriInfo.py"
  "src/CITIUS_Control_Communication/msg/_msg_ctrlRearCamera.py"
  "src/CITIUS_Control_Communication/msg/_msg_electricInfo.py"
  "src/CITIUS_Control_Communication/msg/_msg_rearCameraInfo.py"
  "src/CITIUS_Control_Communication/msg/_msg_command.py"
  "src/CITIUS_Control_Communication/msg/_msg_vehicleInfo.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)