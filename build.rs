/*
extern crate protobuf_codegen_pure;

fn main() {
    protobuf_codegen_pure::run(protobuf_codegen_pure::Args {
        out_dir: "/home/michal/Workspace/CPS/pixhawk-proxy/protos",
        input: &["/home/michal/Workspace/CPS/pixhawk-proxy/protos/mavlink.proto"],
        includes: &["/home/michal/Workspace/CPS/pixhawk-proxy/protos"],
        customize: protobuf_codegen_pure::Customize {
            ..Default::default()
        },
    }).expect("protoc");
}
*/
extern crate prost_build;

fn main() {
    prost_build::compile_protos(&["src/common.proto"],
                                &["src/"]).unwrap();
}