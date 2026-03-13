# comms

ROS2 package responsible for receiving UDP datagrams from the rover's embedded boards, deserializing them, and publishing the data onto ROS topics.

---

## Architecture Overview

```
Embedded Board
    │
    │  UDP datagram (raw PBEnvelope bytes)
    ▼
UdpForwarderNode (:5000)
    │
    ├── Forward raw datagram ──► dst_ip:6000  (e.g. basestation)
    ├── Forward raw datagram ──► dst_ip:6001  (e.g. logger)
    │
    └── Parse PBEnvelope
            │
            ├── payload_case = kImuInfo ──► ImuHandler ──► /imu_data  [comms/ImuSensorInformation]
            └── payload_case = kGpsInfo ──► GpsHandler ──► /gps_data  [comms/SensorBoardGPSInfo]
```

---

## Wire Format

All UDP datagrams are serialized [`PBEnvelope`](https://github.com/RoboTeamTwente/ERC-Protobufs) protobufs with no custom framing header.

```proto
message PBEnvelope {
  oneof payload {
    SensorBoardIMUInfo  imu_info = 13;
    SensorBoardGPSInfo  gps_info = 12;
    // ... other board messages
  }
}
```

The `oneof` makes the datagram self-describing — `payload_case()` identifies the message type without any extra header.

---

## Node: `comms_node`

**Parameters** (set via ROS2 params or launch file):

| Parameter      | Default       | Description                        |
|----------------|---------------|------------------------------------|
| `listen_port`  | `5000`        | UDP port to bind and listen on     |
| `dst_ip`       | `127.0.0.1`   | IP to forward raw datagrams to     |
| `dst_a_port`   | `6000`        | First forwarding destination port  |
| `dst_b_port`   | `6001`        | Second forwarding destination port |

**Published topics:**

| Topic      | Message Type                    | Description              |
|------------|---------------------------------|--------------------------|
| `/imu_data`| `comms/ImuSensorInformation`    | IMU accel, gyro, mag     |
| `/gps_data`| `comms/SensorBoardGPSInfo`      | GPS coordinates, fix info|

---

## Adding a New Handler

1. Add a ROS message in `msg/MyMessage.msg` and register it in `CMakeLists.txt` under `rosidl_generate_interfaces`.
2. Create `include/comms/udp/handlers/my_handler.hpp` (declaration) and `src/handlers/my_handler.cpp` (implementation). Follow the pattern of `ImuHandler` / `GpsHandler`.
3. Add `src/handlers/my_handler.cpp` to `comms_node` sources in `CMakeLists.txt`.
4. Include the handler in `udp_forwarder_node.cpp` and register it:
   ```cpp
   handlers_.emplace(
     static_cast<int>(PBEnvelope::kMyField),
     std::make_unique<MyHandler>(this, "my_topic", 10));
   ```

---

## Building

```bash
colcon build --symlink-install --packages-select comms
source install/setup.bash
```

> **Note:** Protobufs are fetched automatically from [ERC-Protobufs](https://github.com/RoboTeamTwente/ERC-Protobufs) via CMake `FetchContent` on first build.
> If you update `GIT_TAG`, delete `build/comms/_deps/erc_protobufs-*` to force a clean re-fetch.

---

## Running

**Start the node:**
```bash
ros2 run comms comms_node
```

**Test with the UDP client** (sends a single test datagram to `:5000`):
```bash
# Send an IMU envelope (default)
ros2 run comms udp_client 127.0.0.1 0 imu

# Send a GPS envelope
ros2 run comms udp_client 127.0.0.1 0 gps
```

Client argument order: `<server_ip> <local_port (0 = any)> <payload_type>`

**Verify published data:**
```bash
ros2 topic echo /imu_data
ros2 topic echo /gps_data
```

---

## File Structure

```
comms/
├── assets/
│   ├── udp_client.cpp              # Test client — sends a single PBEnvelope over UDP
│   └── test_payloads/
│       ├── imu_payload.hpp/.cpp    # Factory: builds a test IMU PBEnvelope
│       └── gps_payload.hpp/.cpp    # Factory: builds a test GPS PBEnvelope
├── include/comms/udp/
│   ├── handler.hpp                 # Abstract Handler base class
│   ├── utils.hpp                   # Shared utilities (clamp_u8)
│   └── handlers/
│       ├── imu_handler.hpp
│       └── gps_handler.hpp
├── msg/
│   ├── SensorState.msg
│   ├── ImuSensorInformation.msg
│   └── SensorBoardGPSInfo.msg
└── src/
    ├── udp_forwarder_node.cpp      # Main ROS2 node
    └── handlers/
        ├── imu_handler.cpp
        └── gps_handler.cpp
```
