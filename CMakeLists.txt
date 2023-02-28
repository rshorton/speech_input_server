cmake_minimum_required(VERSION 3.5)
project(speech_input_server)

# Set MS_SPEECH_SDK_PATH env var to point to ms voice sdk
# ex:  export MS_SPEECH_SDK_PATH=/home/scott/ms_speech/speechsdk
if(DEFINED ENV{SPEECH_SDK_PATH})
  message(NOTICE "SPEECH_SDK_PATH= $ENV{SPEECH_SDK_PATH}")
else()
  message(FATAL_ERROR "Please create env var SPEECH_SDK_PATH and set to path of MS SPEECH SDK directory.")
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(speech_action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

set(client_library_name speech_input_client)

add_library(${client_library_name} SHARED
  src/speech_action_client.cpp)

target_include_directories(${client_library_name}  PRIVATE
  <BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  <INSTALL_INTERFACE:include>)

ament_target_dependencies(${client_library_name}
  "speech_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs")

rclcpp_components_register_node(${client_library_name} PLUGIN "speech_input_action_server::RecognizeActionClient" EXECUTABLE speech_input_action_client)
install(TARGETS
  ${client_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

set(server_library_name speech_input_server)

add_library(${server_library_name} SHARED
  src/speech_action_server.cpp
  src/speech_input_proc.cpp
  src/respeaker_pixel_ring.cpp)

target_include_directories(${server_library_name} PRIVATE
  <BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  <INSTALL_INTERFACE:include>)

message( STATUS "Running on: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

include_directories(
  $ENV{SPEECH_SDK_PATH}/include/cxx_api
  $ENV{SPEECH_SDK_PATH}/include/c_api)

if( ${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "aarch64" )
  target_link_libraries(${server_library_name}
    $ENV{SPEECH_SDK_PATH}/lib/x64/libMicrosoft.CognitiveServices.Speech.core.so
 	  -lusb-1.0)
else()
	#x86
  target_link_libraries(${server_library_name}
    $ENV{SPEECH_SDK_PATH}/lib/x64/libMicrosoft.CognitiveServices.Speech.core.so
    -lcrypto
    -l:libasound.so.2
   	-lusb-1.0)
endif()

ament_target_dependencies(${server_library_name}
  "speech_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs")

rclcpp_components_register_node(${server_library_name} PLUGIN "speech_input_action_server::SpeechInputActionServer" EXECUTABLE speech_input_action_server)

install(TARGETS
  ${server_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
