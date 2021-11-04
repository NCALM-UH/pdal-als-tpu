find_package(PDAL 2.3.0 REQUIRED CONFIG)

add_library(pdal_plugin_filter_als_tpu SHARED ALS_TPU.cpp)

target_link_libraries(pdal_plugin_filter_als_tpu PRIVATE ${PDAL_LIBRARIES})
target_include_directories(pdal_plugin_filter_als_tpu PRIVATE ${PDAL_INCLUDE_DIRS})
target_link_directories(pdal_plugin_filter_als_tpu PRIVATE ${PDAL_LIBRARY_DIRS})

install(TARGETS pdal_plugin_filter_als_tpu LIBRARY DESTINATION lib)