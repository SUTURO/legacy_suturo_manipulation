FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/my_controller_pkg/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/my_controller_pkg/srv/__init__.py"
  "../src/my_controller_pkg/srv/_SetAmplitude.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
