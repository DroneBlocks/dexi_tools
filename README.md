# DEXI Tools

A ROS2 package providing system utilities for DEXI drones, including firmware flashing capabilities.

## Overview

The `dexi_tools` package provides a ROS2 service for flashing PX4 firmware to ARK flight controllers. The package includes all necessary scripts and dependencies, making it self-contained and easy to deploy.

## Features

- **Firmware Flashing Service**: ROS2 service for uploading PX4 firmware (.px4 files)
- **Real-time Progress Updates**: Progress messages published to ROS topic for web UI integration
- **Self-contained**: All required scripts (px_uploader.py, reset_fmu_wait_bl.py) included
- **Error Handling**: Comprehensive validation and error reporting
- **Low Resource Usage**: Idle node consumes minimal CPU/memory

## Installation

1. Add to your ROS2 workspace via `dexi.repos`:
```bash
vcs import src < dexi.repos
```

2. Build the package:
```bash
colcon build --packages-select dexi_tools
source install/setup.bash
```

## Usage

### Starting the Node

**Option 1: Launch file (recommended)**
```bash
ros2 launch dexi_tools firmware_flash.launch.py
```

**Option 2: Direct node execution**
```bash
ros2 run dexi_tools firmware_flash_node
```

### Command Line Service Calls

**Flash firmware using ROS2 service:**
```bash
# Flash firmware file
ros2 service call /flash_firmware dexi_interfaces/srv/FlashFirmware "{firmware_path: '/tmp/ark_fmu-v6x_default.px4'}"
```

**Monitor progress in real-time:**
```bash
# Subscribe to progress updates
ros2 topic echo /firmware_flash_progress
```

### Complete Example Workflow

1. **Start the firmware flash node:**
```bash
ros2 launch dexi_tools firmware_flash.launch.py
```

2. **In another terminal, monitor progress:**
```bash
ros2 topic echo /firmware_flash_progress
```

3. **In a third terminal, trigger the flash:**
```bash
# Download firmware first (example)
wget -O /tmp/ark_fmu-v6x_default.px4 https://github.com/PX4/PX4-Autopilot/releases/download/v1.16.0/ark_fmu-v6x_default.px4

# Flash the firmware
ros2 service call /flash_firmware dexi_interfaces/srv/FlashFirmware "{firmware_path: '/tmp/ark_fmu-v6x_default.px4'}"
```

4. **Monitor the output in terminal 2:**
```
data: "Starting firmware flash process..."
data: "Firmware file: /tmp/ark_fmu-v6x_default.px4"
data: "Flashing firmware: /tmp/ark_fmu-v6x_default.px4"
data: "/dev/ttyACM0"
data: "Stopping mavlink-router..."
data: "Resetting FMU..."
data: "Uploading firmware..."
data: "Upload complete"
data: "Starting services..."
data: "✅ Firmware flash completed successfully!"
```

### Service Response Format

The `/flash_firmware` service returns:
```yaml
success: true/false          # Whether flash was successful
message: "Status message"    # Detailed status or error description
exit_code: 0                # Exit code from flash process (0 = success)
```

## Docker/Simulation Testing

For testing in Docker containers or environments without ARK hardware:

**Start in simulation mode:**
```bash
ros2 launch dexi_tools firmware_flash_simulation.launch.py
```

**Or manually with parameter:**
```bash
ros2 run dexi_tools firmware_flash_node --ros-args -p simulation_mode:=true
```

**Test with dummy firmware file:**
```bash
# Create a test firmware file
echo "dummy firmware" > /tmp/test_firmware.px4

# Test the service
ros2 service call /flash_firmware dexi_interfaces/srv/FlashFirmware "{firmware_path: '/tmp/test_firmware.px4'}"
```

**Simulation mode features:**
- No hardware dependencies
- Simulates realistic flash timing (~7 seconds)
- Publishes progress messages identical to real hardware
- Perfect for web UI development and testing

## Performance Characteristics

### Resource Usage (Idle)
- **CPU**: ~0.1% (minimal background processing)
- **Memory**: ~15-20MB (typical Python ROS2 node)
- **Network**: No continuous network activity
- **Disk I/O**: None when idle

### Resource Usage (During Flash)
- **CPU**: ~5-15% (subprocess execution and I/O)
- **Memory**: ~20-30MB (temporary buffers)
- **Duration**: ~30-60 seconds typical flash time

The node is designed to be left running continuously with minimal system impact.

## Integration with Web UI

### JavaScript/TypeScript Example (using roslib.js)

```typescript
// Connect to ROS bridge
const ros = new ROSLIB.Ros({ url: 'ws://192.168.7.2:9090' });

// Create service client
const flashService = new ROSLIB.Service({
  ros: ros,
  name: '/flash_firmware',
  serviceType: 'dexi_interfaces/FlashFirmware'
});

// Subscribe to progress updates
const progressTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/firmware_flash_progress',
  messageType: 'std_msgs/String'
});

progressTopic.subscribe((message) => {
  console.log('Flash progress:', message.data);
  // Update UI with progress
});

// Call flash service
const request = new ROSLIB.ServiceRequest({
  firmware_path: '/tmp/uploaded_firmware.px4'
});

flashService.callService(request, (result) => {
  if (result.success) {
    console.log('Flash successful:', result.message);
  } else {
    console.error('Flash failed:', result.message);
  }
});
```

## Requirements

### Hardware
- ARK flight controller connected via USB
- Device appears as `/dev/serial/by-id/*ARK*`

### Software Dependencies
- ROS2 (Jazzy or later)
- Python 3.8+
- systemd (for service management)
- sudo access (for service stop/start)

### System Services
- `mavlink-router` (stopped during flash)
- `logloader.service` (restarted after flash)

## Troubleshooting

### Common Issues

**"ARK device not found"**
```bash
# Check if device is connected
ls -la /dev/serial/by-id/*ARK*

# Check USB connections
lsusb | grep -i ark
```

**"Permission denied"**
```bash
# Ensure user has sudo access
sudo whoami

# Check script permissions
ls -la $(ros2 pkg prefix dexi_tools)/share/dexi_tools/scripts/
```

**"Firmware file not found"**
```bash
# Verify file exists and has correct extension
ls -la /path/to/firmware.px4
file /path/to/firmware.px4
```

### Debug Commands

```bash
# Check if node is running
ros2 node list | grep firmware_flash

# List available services
ros2 service list | grep flash

# Check service type
ros2 service type /flash_firmware

# Monitor all topics
ros2 topic list | grep firmware
```

## Contributing

When contributing to this package:

1. Test firmware flashing with actual hardware
2. Verify progress messages are informative
3. Ensure proper error handling
4. Update documentation for new features

## License

MIT License - See LICENSE file for details.