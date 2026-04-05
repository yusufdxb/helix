# Scenario A: /rosout Observation Under Load

## Protocol
**Objective**: Determine whether publishing harmless test messages to a novel topic causes
any observable reaction in the GO2 robot's /rosout log stream.

**Safety classification**: LOW RISK -- publishing to a novel topic (/helix_test_topic) that
no robot node subscribes to. No control topics are touched.

### Steps Executed
1. Started bag recording on /rosout and /helix_test_topic (30s duration)
2. Published 20 test messages to /helix_test_topic at 1-second intervals using
   `ros2 topic pub --once /helix_test_topic std_msgs/msg/String "data: 'helix_test_msg_N'"`
3. Waited for bag recording to complete
4. Verified bag with `ros2 bag info`

### Results
- **Bag captured**: 15 of 20 test messages, 7 /rosout messages
- **Key finding**: All 7 /rosout messages originated from the rosbag2_recorder node itself,
  NOT from any GO2 node reacting to the test topic
- **Conclusion**: External topic injection is completely isolated from robot internals.
  The GO2 does not react to or log messages about unknown topics appearing on the DDS bus.

### Implications for HELIX
- HELIX's /rosout log parser will only see robot-internal events (node start/stop, errors, warnings)
- External perturbations that don't touch robot topics will be invisible to /rosout monitoring
- To detect external network issues, HELIX would need rate monitoring (topic hz deviation)
  or latency monitoring, not just log parsing

### Evidence
- Bag: `bags/helix_rosout_perturbation/`
- Sidecar: `bags/helix_rosout_perturbation/metadata_sidecar.yaml`
