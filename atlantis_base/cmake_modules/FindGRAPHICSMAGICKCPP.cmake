include(FindPackageHandleStandardArgs)

find_path(GRAPHICSMAGICKCPP_INCLUDE_DIRS
  NAMES "Magick++.h"
  PATH_SUFFIXES GraphicsMagick)

find_library(GRAPHICSMAGICKCPP_LIBRARIES
  NAMES "GraphicsMagick++" "graphicsmagick")

find_package_handle_standard_args(
  GRAPHICSMAGICKCPP
  GRAPHICSMAGICKCPP_LIBRARIES
  GRAPHICSMAGICKCPP_INCLUDE_DIRS)