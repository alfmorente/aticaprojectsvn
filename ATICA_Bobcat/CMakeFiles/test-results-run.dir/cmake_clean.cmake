FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/Driving_Bobcat/msg"
  "src/Driving_Bobcat/srv"
  "CMakeFiles/test-results-run"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/test-results-run.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
