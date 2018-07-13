extern crate prost_build;

fn main() {
    let mut config = prost_build::Config::new();
    config.type_attribute(".", "#[derive(Serialize)]");
    config.compile_protos(&["protos/mavlink_common.proto"],
                                &["protos/"]).unwrap();
}