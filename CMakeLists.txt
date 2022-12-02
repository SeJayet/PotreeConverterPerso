cmake_minimum_required(VERSION 3.24)
set(CMAKE_SUPPRESS_REGENERATION true)

project(PotreeConverter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)


################################################################################
# Check for nlohmann_json
################################################################################

find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(main PRIVATE nlohmann_json::nlohmann_json)

################################################################################
# Check for laszip
################################################################################

find_package(laszip CONFIG REQUIRED)

################################################################################
# Check for brotli
################################################################################

find_package(unofficial-brotli CONFIG REQUIRED)
target_link_libraries(main PRIVATE unofficial::brotli::brotlidec unofficial::brotli::brotlienc unofficial::brotli::brotlicommon)

#message(${PROJECT_SOURCE_DIR})

set(HEADER_FILES  
	./Converter/include/Attributes.h
	./Converter/include/chunker_countsort_laszip.h
	./Converter/include/ChunkRefiner.h
	./Converter/include/ConcurrentWriter.h
	./Converter/include/converter_utils.h
	./Converter/include/indexer.h
	./Converter/include/prototyping.h
	./Converter/include/sampler_poisson.h
	./Converter/include/sampler_poisson_average.h
	./Converter/include/sampler_random.h
	./Converter/include/structures.h
	./Converter/include/Vector3.h
	./Converter/include/PotreeConverter.h
	./Converter/include/logger.h
	./Converter/modules/LasLoader/LasLoader.h
	./Converter/modules/unsuck/unsuck.hpp
)

add_executable(PotreeConverter 
	./Converter/src/chunker_countsort_laszip.cpp
	./Converter/src/indexer.cpp 
	./Converter/src/main.cpp
	./Converter/src/logger.cpp
	./Converter/modules/LasLoader/LasLoader.cpp
	./Converter/modules/unsuck/unsuck_platform_specific.cpp
	${HEADER_FILES}
)

target_include_directories(PotreeConverter PRIVATE "./Converter/include")
target_include_directories(PotreeConverter PRIVATE "./Converter/modules")
target_include_directories(PotreeConverter PRIVATE "./Converter/libs")


if (UNIX)
	find_package(Threads REQUIRED)
	find_package(TBB REQUIRED)
	
	target_link_libraries(${PROJECT_NAME} Threads::Threads)
	target_link_libraries(${PROJECT_NAME} tbb)


	#SET(CMAKE_CXX_FLAGS "-pthread -ltbb")
endif (UNIX)

###############################################
# COPY LICENSE FILES TO BINARY DIRECTORY
###############################################
add_custom_command(
	TARGET PotreeConverter POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy
		${PROJECT_SOURCE_DIR}/LICENSE
		$<TARGET_FILE_DIR:${PROJECT_NAME}>/licenses/license_potree_converter.txt)

add_custom_command(
	TARGET PotreeConverter POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E copy
		${PROJECT_SOURCE_DIR}/README.md
		$<TARGET_FILE_DIR:${PROJECT_NAME}>/README.md)

