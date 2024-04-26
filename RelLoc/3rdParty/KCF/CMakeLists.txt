project(KCF)

include_directories(${OPENCV_INCLUDE_DIR})

set(KCF_SOURCE_FILES
	ffttools.hpp
	fhog.cpp
	fhog.hpp
	kcftracker.cpp
	kcftracker.hpp
	labdata.hpp
	recttools.hpp
#	tracker_test.cpp
	tracker.h
   )
   
set_source_files_properties(${features_files_sources} PROPERTIES LANGUAGE CXX)

add_library( kcf SHARED ${KCF_SOURCE_FILES})

target_link_libraries(kcf ${OpenCV_LIBS})

