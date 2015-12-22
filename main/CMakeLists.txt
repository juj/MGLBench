cmake_minimum_required(VERSION 2.8)

file(GLOB sourceFiles *.cpp)

if (MATH_SSE2)
	message(STATUS "Enabling SSE2 Release")
	add_definitions(-DMATH_SSE2)
	add_definitions(-DMATH_ENABLE_INSECURE_OPTIMIZATIONS)
	add_definitions(-DMATH_SILENT_ASSUME)
	add_definitions(-DRELEASE)
	add_definitions(-DOPTIMIZED_RELEASE)

	if (EMSCRIPTEN)
		add_definitions(-msse2)
	endif()
endif()

if (MATH_SSE)
	message(STATUS "Enabling SSE Release")
	add_definitions(-DMATH_SSE)
	add_definitions(-DMATH_ENABLE_INSECURE_OPTIMIZATIONS)
	add_definitions(-DMATH_SILENT_ASSUME)
	add_definitions(-DRELEASE)
	add_definitions(-DOPTIMIZED_RELEASE)

	if (EMSCRIPTEN)
		add_definitions(-msse)
	endif()
endif()

if (EMSCRIPTEN)
	set(CMAKE_EXECUTABLE_SUFFIX ".html")
endif()

include_directories(../MathGeoLib/src/)
add_executable(MGLBench ${sourceFiles})
target_link_libraries(MGLBench MathGeoLib)
