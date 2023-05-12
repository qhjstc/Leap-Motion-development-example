cmake_minimum_required(VERSION 3.16.3)

project(leap_example VERSION "1.0.0" LANGUAGES C)

if (WIN32)
    set(ULTRALEAP_PATH_ROOT "D:/Program\ Files/Ultraleap")
elseif (UNIX)
    set(ULTRALEAP_PATH_ROOT "/usr/share/doc/ultraleap-hand-tracking-service/lib/cmake/LeapSDK")
endif()

find_package(LeapSDK
        5
        REQUIRED
        PATHS
        "${ULTRALEAP_PATH_ROOT}")

if (UNIX)
    find_package(Threads REQUIRED)
endif (UNIX)

add_executable(
        leap_example
        "samples/main.c")

target_link_libraries(
        leap_example
        PRIVATE
        LeapSDK::LeapC)

get_target_property(
        LEAPC_IMPORTED_CONFIG
        LeapSDK::LeapC
        IMPORTED_CONFIGURATIONS
)

get_target_property(
        LEAPC_SHARED_LIB_PATH
        LeapSDK::LeapC
        IMPORTED_LOCATION_${LEAPC_IMPORTED_CONFIG}
)

add_custom_command(
        TARGET
        leap_example
        POST_BUILD
        COMMAND
        ${CMAKE_COMMAND} -E copy
        ${LEAPC_SHARED_LIB_PATH}
        $<TARGET_FILE_DIR:leap_example>)

add_library(
        libExampleConnection
        OBJECT
        "samples/ExampleConnection.c")

target_link_libraries(
        libExampleConnection
        PUBLIC
        LeapSDK::LeapC)

if (UNIX)
    target_link_libraries(
            libExampleConnection
            PRIVATE
            Threads::Threads)
endif()

target_include_directories(
        libExampleConnection
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})

#添加头文件包含路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/functions-inc)

file(GLOB functions_src_codes functions-src/*.c)
add_library(
        libFunction
        STATIC
        ${functions_src_codes})

# Add targets for each sample file.
function(add_sample sample_name sample_source_file)

    add_executable(${sample_name} ${sample_source_file})
    set_property(TARGET ${sample_name} PROPERTY FOLDER "Samples")
    target_link_libraries(${sample_name} PUBLIC libExampleConnection libFunction)

endfunction()


# Add project samples

add_sample("main" "samples/main.c")
add_sample("main_get_image" "samples/main_get_image.c")
add_sample("CallbackSample" "samples/CallbackSample.c")
add_sample("ImageSample" "samples/ImageSample.c")
add_sample("PollingSample" "samples/PollingSample.c")
add_sample("InterpolationSample" "samples/InterpolationSample.c")
add_sample("RecordPlayBackSample" "samples/RecordPlayBackSample.c")
if(NOT ANDROID)
    add_sample("MultiDeviceSample" "samples/MultiDeviceSample.c")
endif()

