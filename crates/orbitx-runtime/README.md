# orbitx-runtime

产品仿真主进程（无 GUI）。权威设计：[`docs/RUNTIME.md`](../../docs/RUNTIME.md)；黑匣子格式：[`docs/FLIGHT_RECORDER.md`](../../docs/FLIGHT_RECORDER.md)。

## 现状（P4.2 + P4.3）

- `clap` 启动参数
- `RuntimeService`：`std::thread`（真步进 + Controller）
- `CommsService`：本机 Zenoh + SHM + protobuf（P4.3 ✅；禁跨设备）
- IO tokio：`tracing` Log + FlightRecorder L1 入队（CBOR 段文件 → P6）
- `flume` channel + `ShutdownFlag` 有序停机

## 运行

```bash
cargo run -p orbitx-runtime -- --help
cargo run -p orbitx-runtime -- --drive self-paced --sim-dt 20
```

## 测试

```bash
cargo test -p orbitx-runtime
```
