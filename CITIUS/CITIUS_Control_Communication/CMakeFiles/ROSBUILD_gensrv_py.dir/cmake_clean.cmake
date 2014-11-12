FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/CITIUS_Control_Communication/msg"
  "src/CITIUS_Control_Communication/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/CITIUS_Control_Communication/srv/__init__.py"
  "src/CITIUS_Control_Communication/srv/_srv_zoomCommand.py"
  "src/CITIUS_Control_Communication/srv/_srv_zoomDirect.py"
  "src/CITIUS_Control_Communication/srv/_srv_dzoom.py"
  "src/CITIUS_Control_Communication/srv/_srv_panAbsolutePosition.py"
  "src/CITIUS_Control_Communication/srv/_srv_shoot.py"
  "src/CITIUS_Control_Communication/srv/_srv_focusDirect.py"
  "src/CITIUS_Control_Communication/srv/_srv_nodeStatus.py"
  "src/CITIUS_Control_Communication/srv/_srv_tiltRate.py"
  "src/CITIUS_Control_Communication/srv/_srv_autofocusMode.py"
  "src/CITIUS_Control_Communication/srv/_srv_polarity.py"
  "src/CITIUS_Control_Communication/srv/_srv_panRate.py"
  "src/CITIUS_Control_Communication/srv/_srv_vehicleStatus.py"
  "src/CITIUS_Control_Communication/srv/_srv_tiltAbsolutePosition.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
