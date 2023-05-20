include_directories($ENV{LMINI_ROOT}/libs/include)
include_directories($ENV{LMINI_ROOT}/libs/include/msg)
# include_directories($ENV{LMINI_ROOT}/libs_3rd/include)
# include_directories($ENV{LMINI_ROOT}/libs_3rd/include)
link_directories($ENV{LMINI_ROOT}/libs/x86)
set(ltool_libs ltools lmw)

file(GLOB_RECURSE proto_file $ENV{LMINI_ROOT}/libs/include/msg/*.cpp)
