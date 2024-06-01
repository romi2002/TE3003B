project(puzzlebot_proto)
set (CMAKE_CXX_STANDARD 14)

set (Protobuf_DEBUG ON)
find_package(Protobuf REQUIRED)

include(FetchContent)
FetchContent_Declare(
  gRPC
  GIT_REPOSITORY https://github.com/grpc/grpc
  GIT_TAG        v1.30.2
)
set(FETCHCONTENT_QUIET OFF)
FetchContent_MakeAvailable(gRPC)

set(_PROTOBUF_LIBPROTOBUF libprotobuf)
set(_REFLECTION grpc++_reflection)
set(_PROTOBUF_PROTOC $<TARGET_FILE:protoc>)
set(_GRPC_GRPCPP grpc++)
set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:grpc_cpp_plugin>)

file(GLOB_RECURSE PROTO_SOURCES "*.cc")
file(GLOB_RECURSE GOOGLE_PROTO_FILES
    google/api/annotations.proto
    google/api/http.proto
)

add_library(puzzlebot_proto ${PROTO_SOURCES} ${GOOGLE_PROTO_FILES})
target_link_libraries(puzzlebot_proto
    PUBLIC
        protobuf::libprotobuf
        grpc++
)

target_include_directories(puzzlebot_proto PUBLIC 
    ${CMAKE_CURRENT_BINARY_DIR}
)
