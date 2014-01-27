FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/ltm_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ltm_msgs/msg/__init__.py"
  "../src/ltm_msgs/msg/_DModel.py"
  "../src/ltm_msgs/msg/_Edge.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
