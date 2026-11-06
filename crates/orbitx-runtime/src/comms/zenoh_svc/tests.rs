use super::*;
use crate::channel::{send_slice_keep_latest, RuntimeChannels};

// —— 配置契约（自 zenoh_svc.rs 内联测试迁出） ——

fn assert_local_transport(config: &Config) {
    assert_eq!(
        config
            .get_json("transport/shared_memory/transport_optimization/message_size_threshold")
            .unwrap(),
        "512"
    );
    assert_eq!(
        config.get_json("transport/shared_memory/mode").unwrap(),
        r#""init""#
    );
    assert_eq!(
        config.get_json("transport/unicast/lowlatency").unwrap(),
        "true"
    );
    assert_eq!(
        config.get_json("transport/unicast/qos/enabled").unwrap(),
        "false"
    );
}

#[test]
fn host_and_client_share_lowlatency_shm() {
    let host = build_host_config("local").unwrap();
    let client = build_client_config("local").unwrap();
    assert_local_transport(&host);
    assert_local_transport(&client);
    assert_eq!(
        host.get_json("listen/endpoints").unwrap(),
        r#"["tcp/127.0.0.1:17447"]"#
    );
    assert_eq!(
        client.get_json("connect/endpoints").unwrap(),
        r#"["tcp/127.0.0.1:17447"]"#
    );
}

#[test]
fn session_id_maps_to_stable_local_port() {
    // 同 endpoint 稳定；异 endpoint 落在 17000..19000 本机段。
    let a = local_locator("session-a");
    let a2 = local_locator("session-a");
    assert_eq!(a, a2);
    let port: u32 = a.trim_start_matches("tcp/127.0.0.1:").parse().unwrap();
    assert!((17000..19000).contains(&port));
}

// —— inbound_from_proto：proto → RuntimeInbound 映射 ——

// 注意：proto 侧 InputCmd/SessionCmd 与 runtime 侧 crate::input::{InputCmd,
// SessionCmd} 同名（super::* 导入的是后者）——proto 类型须全限定。

#[test]
fn input_kind_maps_to_runtime_input_cmd() {
    let msg = Inbound {
        payload: Some(InboundPayload::Input(orbitx_protocol::InputCmd {
            kind: Some(InputKind::SetThrottle(orbitx_protocol::SetThrottle {
                level: 0.75,
            })),
        })),
    };
    match inbound_from_proto(msg) {
        Some(RuntimeInbound::Input(InputCmd::SetThrottle { level })) => {
            assert!((level - 0.75).abs() < 1e-12);
        }
        other => panic!("unexpected {other:?}"),
    }
}

#[test]
fn session_kinds_map_to_runtime_session_cmd() {
    let step = Inbound {
        payload: Some(InboundPayload::Session(orbitx_protocol::SessionCmd {
            kind: Some(SessionKind::Step(orbitx_protocol::Step { n: 3 })),
        })),
    };
    assert!(matches!(
        inbound_from_proto(step),
        Some(RuntimeInbound::Session(SessionCmd::Step { n: 3 }))
    ));

    let warp = Inbound {
        payload: Some(InboundPayload::Session(orbitx_protocol::SessionCmd {
            kind: Some(SessionKind::SetWarp(orbitx_protocol::SetWarp {
                scale: 2.0,
            })),
        })),
    };
    assert!(matches!(
        inbound_from_proto(warp),
        Some(RuntimeInbound::Session(SessionCmd::SetWarp { scale }))
            if (scale - 2.0).abs() < 1e-12
    ));

    let launch = Inbound {
        payload: Some(InboundPayload::Session(orbitx_protocol::SessionCmd {
            kind: Some(SessionKind::Launch(orbitx_protocol::Launch {})),
        })),
    };
    assert!(matches!(
        inbound_from_proto(launch),
        Some(RuntimeInbound::Session(SessionCmd::Launch))
    ));
}

#[test]
fn missing_payload_or_kind_maps_to_none() {
    assert!(inbound_from_proto(Inbound { payload: None }).is_none());
    let input_no_kind = Inbound {
        payload: Some(InboundPayload::Input(orbitx_protocol::InputCmd {
            kind: None,
        })),
    };
    assert!(inbound_from_proto(input_no_kind).is_none());
    let session_no_kind = Inbound {
        payload: Some(InboundPayload::Session(orbitx_protocol::SessionCmd {
            kind: None,
        })),
    };
    assert!(inbound_from_proto(session_no_kind).is_none());
}

// —— collect_latest_slice / is_new_slice：keep-latest 与 P4.5 判新不变量 ——

fn slice_at(step: u64) -> Arc<Slice> {
    Arc::new(Slice {
        step_index: step,
        ..Default::default()
    })
}

#[test]
fn collect_drains_queue_to_latest() {
    let ch = RuntimeChannels::bounded();
    let handles = CommsHandles::from_channels(&ch);
    for i in 1..=3 {
        send_slice_keep_latest(&ch.slice_bus, slice_at(i));
    }
    let got = collect_latest_slice(&handles, None).expect("drained");
    assert_eq!(got.step_index, 3);
    // 排空后再次收集为空（不重发）。
    assert!(collect_latest_slice(&handles, None).is_none());
}

#[test]
fn collect_slot_wins_over_queue_tail_on_full() {
    let ch = RuntimeChannels::bounded();
    let handles = CommsHandles::from_channels(&ch);
    // 队列容量 SLICE_CAPACITY=8：塞满后下一片进旁路 latest 槽。
    for i in 1..=8 {
        send_slice_keep_latest(&ch.slice_bus, slice_at(i));
    }
    send_slice_keep_latest(&ch.slice_bus, slice_at(9));
    // 排空队列至 8，槽（9）覆盖队尾 → 返回 9。
    let got = collect_latest_slice(&handles, None).expect("latest");
    assert_eq!(got.step_index, 9);
    // 一次收集即消费全部（队列排空 + 槽取走）——不重发。
    assert!(collect_latest_slice(&handles, None).is_none());
}

#[test]
fn collect_head_from_wake_participates() {
    // select 分支醒来时已持有的头一片参与排空比较（模拟 recv_async 返回值）。
    let ch = RuntimeChannels::bounded();
    let handles = CommsHandles::from_channels(&ch);
    send_slice_keep_latest(&ch.slice_bus, slice_at(5));
    send_slice_keep_latest(&ch.slice_bus, slice_at(6));
    let head = handles.slice_rx.try_recv().expect("head");
    let got = collect_latest_slice(&handles, Some(head)).expect("latest");
    assert_eq!(got.step_index, 6);
}

#[test]
fn same_arc_is_not_new_but_new_allocation_is() {
    let a = slice_at(1);
    // 同一 Arc（clone 共享指针）→ 非新片，禁止重发。
    assert!(!is_new_slice(&Some(a.clone()), &a));
    // 不同分配（即使内容相同）→ 按新片发布（与原轮询版 ptr_eq 语义一致）。
    let b = slice_at(1);
    assert!(is_new_slice(&Some(a), &b));
    assert!(is_new_slice(&None, &b));
}
