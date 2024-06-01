package main

import (
        "context"
        "flag"
        "fmt"
        "net/http"
        "github.com/golang/glog"

        "google.golang.org/grpc"
        "github.com/grpc-ecosystem/grpc-gateway/v2/runtime"
        "github.com/rs/cors"
        gw "robotonotos.com/grpc-gateway/proto"

)

var (
        grpcServerEndpoint = flag.String("grpc-server-endpoint", "127.0.0.1:7042", "gRPC svr endpoint")
        gw_port = "8042"

)

func enableCors(h http.Handler) http.Handler {
    return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
        w.Header().Set("Access-Control-Allow-Origin", "*")

        h.ServeHTTP(w, r)
    })
}

func run() error {
        ctx := context.Background()
        ctx, cancel := context.WithCancel(ctx)
        defer cancel()

        mux := runtime.NewServeMux()
        withCors := cors.New(cors.Options{
        AllowOriginFunc:  func(origin string) bool { return true },
        AllowedMethods:   []string{"GET", "POST", "PATCH", "PUT", "DELETE", "OPTIONS"},
        AllowedHeaders:   []string{"ACCEPT", "Authorization", "Content-Type", "X-CSRF-Token"},
        ExposedHeaders:   []string{"Link"},
        AllowCredentials: true,
        MaxAge:           300,
        }).Handler(mux)

        opts := []grpc.DialOption{grpc.WithInsecure()}
        err := gw.RegisterRobotStateHandlerFromEndpoint(ctx, mux, *grpcServerEndpoint, opts)
        if err != nil{
                return err
        }
        return http.ListenAndServe(":"+gw_port, withCors)


}

func main(){
        flag.Parse()
        defer glog.Flush()
        fmt.Println("Starting Gateway at "+ gw_port)
        if err := run(); err != nil{
                glog.Fatal(err)
        }
}