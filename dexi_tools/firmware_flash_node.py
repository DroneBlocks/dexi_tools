#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dexi_interfaces.srv import FlashFirmware
import subprocess
import threading
import os
import time
from .firmware_upload_server import FirmwareUploadServer


class FirmwareFlashNode(Node):
    def __init__(self):
        super().__init__('firmware_flash_node')

        # Create service for firmware flashing
        self.srv = self.create_service(
            FlashFirmware,
            'flash_firmware',
            self.flash_firmware_callback
        )

        # Create publisher for progress updates
        self.progress_pub = self.create_publisher(
            String,
            'firmware_flash_progress',
            10
        )

        # Check for simulation mode
        self.declare_parameter('simulation_mode', False)
        self.simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value

        # Get package share directory for scripts
        from ament_index_python.packages import get_package_share_directory
        self.package_share_dir = get_package_share_directory('dexi_tools')
        self.scripts_dir = os.path.join(self.package_share_dir, 'scripts')

        # Initialize HTTP upload server
        self.declare_parameter('upload_port', 8080)
        self.declare_parameter('upload_dir', '/shared/firmware_uploads')

        upload_port = self.get_parameter('upload_port').get_parameter_value().integer_value
        upload_dir = self.get_parameter('upload_dir').get_parameter_value().string_value

        self.upload_server = FirmwareUploadServer(port=upload_port, upload_dir=upload_dir)
        if self.upload_server.start():
            self.get_logger().info(f'HTTP upload server started on port {upload_port}')
        else:
            self.get_logger().error('Failed to start HTTP upload server')

        self.get_logger().info('Firmware flash node started')
        self.get_logger().info(f'Scripts directory: {self.scripts_dir}')
        self.get_logger().info(f'Simulation mode: {self.simulation_mode}')
        self.get_logger().info(f'Upload directory: {upload_dir}')

    def publish_progress(self, message):
        """Publish progress message to topic"""
        msg = String()
        msg.data = message
        self.progress_pub.publish(msg)
        self.get_logger().info(f'Progress: {message}')

    def simulate_flash_process(self, firmware_path):
        """Simulate firmware flash process for testing"""
        import time

        steps = [
            "Starting firmware flash process...",
            f"Firmware file: {firmware_path}",
            "Flashing firmware: " + firmware_path,
            "Simulated ARK device: /dev/ttyUSB0",
            "Stopping mavlink-router...",
            "Resetting FMU...",
            "Uploading firmware... (simulation)",
            "Progress: 25%",
            "Progress: 50%",
            "Progress: 75%",
            "Progress: 100%",
            "Upload complete (simulated)",
            "Starting services...",
            "✅ Firmware flash completed successfully! (simulation)"
        ]

        for step in steps:
            self.publish_progress(step)
            time.sleep(0.5)  # Simulate processing time

        return True, "Simulated flash completed successfully"

    def flash_firmware_callback(self, request, response):
        """Service callback to flash firmware"""
        try:
            self.get_logger().info(f'Flash firmware request: {request.firmware_path}')

            # Validate firmware file exists
            if not os.path.exists(request.firmware_path):
                response.success = False
                response.message = f"Firmware file not found: {request.firmware_path}"
                response.exit_code = 1
                return response

            # Validate firmware file has .px4 extension
            if not request.firmware_path.endswith('.px4'):
                response.success = False
                response.message = "Firmware file must have .px4 extension"
                response.exit_code = 1
                return response

            if self.simulation_mode:
                # Run simulation instead of actual flash
                self.get_logger().info("Running in simulation mode")
                success, message = self.simulate_flash_process(request.firmware_path)
                response.success = success
                response.message = message
                response.exit_code = 0 if success else 1
                return response

            # Get flash script path
            flash_script = os.path.join(self.scripts_dir, 'flash_px4.sh')

            if not os.path.exists(flash_script):
                response.success = False
                response.message = f"Flash script not found: {flash_script}"
                response.exit_code = 1
                return response

            self.publish_progress("Starting firmware flash process...")
            self.publish_progress(f"Firmware file: {request.firmware_path}")

            # Execute flash script with real-time output
            process = subprocess.Popen([
                flash_script, request.firmware_path
            ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
               universal_newlines=True, bufsize=1)

            # Stream output line by line
            output_lines = []
            for line in iter(process.stdout.readline, ''):
                if line:
                    line = line.strip()
                    output_lines.append(line)
                    self.publish_progress(line)

            # Wait for process to complete
            process.wait()

            # Prepare response
            response.exit_code = process.returncode
            response.success = process.returncode == 0

            if response.success:
                response.message = "Firmware flash completed successfully"
                self.publish_progress("✅ Firmware flash completed successfully!")
            else:
                response.message = f"Firmware flash failed with exit code {process.returncode}"
                self.publish_progress(f"❌ Firmware flash failed with exit code {process.returncode}")

            # Include output in response message
            if output_lines:
                response.message += f"\n\nOutput:\n" + "\n".join(output_lines)

        except Exception as e:
            self.get_logger().error(f'Flash firmware error: {str(e)}')
            response.success = False
            response.message = f"Flash error: {str(e)}"
            response.exit_code = -1
            self.publish_progress(f"❌ Flash error: {str(e)}")

        return response


def main(args=None):
    rclpy.init(args=args)

    firmware_flash_node = FirmwareFlashNode()

    try:
        rclpy.spin(firmware_flash_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the HTTP upload server
        if hasattr(firmware_flash_node, 'upload_server'):
            firmware_flash_node.upload_server.stop()
        firmware_flash_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()