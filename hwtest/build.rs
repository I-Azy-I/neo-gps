fn main() {
    // esp-hal's memory layout.
    println!("cargo:rustc-link-arg=-Tlinkall.x");
    // embedded-test's semihosting-based harness.
    println!("cargo::rustc-link-arg=-Tembedded-test.x");
    println!("cargo::rustc-check-cfg=cfg(rust_analyzer)");

    // Test-suite configuration is baked in at build time (there is no
    // environment on the target), so a changed value must force a rebuild.
    for var in [
        "NEO_GEN",
        "NEO_BAUD",
        "NEO_ALLOW_SAVE",
        "NEO_ALLOW_RESET",
        "NEO_ALLOW_BAUD",
    ] {
        println!("cargo:rerun-if-env-changed={var}");
    }
}
