fn main() {
    // Emits $CARGO_TARGET_DIR/cxxbridge/f3rt-audio-cxx/src/lib.rs.h (+ rust/cxx.h) for MAME and
    // compiles the matching C++ shim into the staticlib.
    cxx_build::bridge("src/lib.rs").std("c++17").compile("f3rt-audio-cxx-bridge");
    println!("cargo:rerun-if-changed=src/lib.rs");
}
