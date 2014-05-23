FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/CITIUS_Control_Manager/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/CITIUS_Control_Manager/srv/__init__.py"
  "../src/CITIUS_Control_Manager/srv/_srv_vehicle.py"
  "../src/CITIUS_Control_Manager/srv/_srv_frontcam.py"
  "../src/CITIUS_Control_Manager/srv/_srv_status.py"
  "../src/CITIUS_Control_Manager/srv/_srv_electric.py"
  "../src/CITIUS_Control_Manager/srv/_srv_rearcam.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
