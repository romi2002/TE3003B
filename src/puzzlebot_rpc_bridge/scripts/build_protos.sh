go get \
    github.com/grpc-ecosystem/grpc-gateway/v2/protoc-gen-grpc-gateway \
    github.com/grpc-ecosystem/grpc-gateway/v2/protoc-gen-openapiv2 \
    google.golang.org/protobuf/cmd/protoc-gen-go \
    google.golang.org/grpc/cmd/protoc-gen-go-grpc

go install \
    github.com/grpc-ecosystem/grpc-gateway/v2/protoc-gen-grpc-gateway \
    github.com/grpc-ecosystem/grpc-gateway/v2/protoc-gen-openapiv2 \
    google.golang.org/protobuf/cmd/protoc-gen-go \
    google.golang.org/grpc/cmd/protoc-gen-go-grpc

go get google.golang.org/grpc/cmd/protoc-gen-go-grpc

# PATH=$PATH:$HOME/go/bin protoc -I . --grpc-gateway_out ./gen/go \
#     --plugin=protoc-gen-grpc-gateway= \
#     --grpc-gateway_opt paths=source_relative \
#     RobotState.proto
PATH=$PATH:$HOME/go/bin protoc --proto_path=proto --go_out=proto --go_opt=paths=source_relative proto/RobotState.proto
PATH=$PATH:$HOME/go/bin protoc -I proto --grpc-gateway_out ./proto \
    --grpc-gateway_opt paths=source_relative \
    proto/RobotState.proto
protoc --proto_path=proto --grpc_out=proto \
 --plugin=protoc_gen_grpc=/usr/src/grpc/cmake/build/grpc_cpp_plugin proto/RobotState.proto

python3 -m grpc_tools.protoc -I=proto --python_out=proto --pyi_out=proto --grpc_python_out=proto proto/RobotState.proto proto/google/api/annotations.proto proto/google/api/http.proto