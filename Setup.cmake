macro(Setup)
    message(STATUS "Running Setup...")

	IF(NOT CMAKE_BUILD_TYPE)
	  SET(CMAKE_BUILD_TYPE Release)
	ENDIF()

	string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)

	# CHECK C++11 SUPPORT
	include(CheckCXXCompilerFlag)
	CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
	if(COMPILER_SUPPORTS_CXX14)
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
	   add_definitions(-DCOMPILEDWITHC14)
	   message(STATUS "Using flag -std=c++14.")	
	else()
	   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. CMake will exit.")
	endif()

	# FIND GLUT
	find_package(GLUT REQUIRED)
	include_directories(${GLUT_INCLUDE_DIRS})
	link_directories(${GLUT_LIBRARY_DIRS})
	add_definitions(${GLUT_DEFINITIONS})
	if(NOT GLUT_FOUND)
		message(FATAL_ERROR "GLUT not found, CMake will exit.")
	endif(NOT GLUT_FOUND)

	# FIND OPENGL
	find_package(OpenGL REQUIRED)
	include_directories(${OpenGL_INCLUDE_DIRS})
	link_directories(${OpenGL_LIBRARY_DIRS})
	add_definitions(${OpenGL_DEFINITIONS})
	if(NOT OPENGL_FOUND)
		message(FATAL_ERROR "OPENGL not found, CMake will exit.")
	endif(NOT OPENGL_FOUND)

	#FIND REQUIRED PACKAGE
	find_package(OpenCV 3.2  REQUIRED )
	find_package(Eigen3 REQUIRED )
	find_package(Pangolin REQUIRED)
	find_package(FREEGLUT QUIET)
    find_package(Threads REQUIRED)

	#FIND AIRSIM DIR
	find_path(AIRSIM_ROOT NAMES AirSim.sln PATHS "../AirSim" "../../AirSim" "../../../AirSim" "../Libs/AirSim")
	if(AIRSIM_ROOT)
		message(STATUS "found AIRSIM_ROOT=${AIRSIM_ROOT}")
		include_directories(
			${AIRSIM_ROOT}/AirLib/include
			${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include
			${AIRSIM_ROOT}/MavLinkCom/include
			${AIRSIM_ROOT}/MavLinkCom/common_utils
		)
	else()
		message(FATAL_ERROR "AIRSIM not found, CMake will exit.")
	endif()

    #SETUP OUTPUT PATHS
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

endmacro(Setup)

