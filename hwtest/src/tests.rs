#[test]
fn last_ubx_payload_before_any_frame() {
    let gps = crate::NeoGps::new(crate::testutil::MockUart::new(Vec::new()));
    assert_eq!(gps.last_ubx_payload(), &[] as &[u8]);
}
