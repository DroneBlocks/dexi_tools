#!/usr/bin/env python3

import os
import time
import threading
import cgi
import json
import shutil
from http.server import HTTPServer, BaseHTTPRequestHandler


class FirmwareUploadHandler(BaseHTTPRequestHandler):
    """HTTP handler for firmware file uploads"""

    def do_POST(self):
        """Handle POST requests for file uploads"""
        try:
            if self.path == '/upload-firmware':
                # Parse the multipart form data
                content_type = self.headers.get('Content-Type')
                if not content_type or not content_type.startswith('multipart/form-data'):
                    self.send_error(400, 'Content-Type must be multipart/form-data')
                    return

                # Parse form data
                form = cgi.FieldStorage(
                    fp=self.rfile,
                    headers=self.headers,
                    environ={
                        'REQUEST_METHOD': 'POST',
                        'CONTENT_TYPE': content_type,
                    }
                )

                # Find the firmware file field
                firmware_field = form['firmware'] if 'firmware' in form else None

                if not firmware_field or not firmware_field.filename:
                    self.send_error(400, 'No firmware file uploaded')
                    return

                # Validate file extension
                if not firmware_field.filename.endswith('.px4'):
                    self.send_error(400, 'File must have .px4 extension')
                    return

                # Create upload directory if it doesn't exist
                upload_dir = getattr(self.server, 'upload_dir', '/shared/firmware_uploads')
                os.makedirs(upload_dir, exist_ok=True)

                # Generate unique filename
                timestamp = int(time.time() * 1000)  # milliseconds
                filename = f"{timestamp}_{firmware_field.filename}"
                file_path = os.path.join(upload_dir, filename)

                # Save the file
                with open(file_path, 'wb') as f:
                    shutil.copyfileobj(firmware_field.file, f)

                # Send success response
                response_data = {
                    'success': True,
                    'message': 'File uploaded successfully',
                    'path': file_path,
                    'filename': filename,
                    'originalName': firmware_field.filename,
                    'size': os.path.getsize(file_path)
                }

                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')  # Enable CORS
                self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                self.end_headers()

                self.wfile.write(json.dumps(response_data).encode('utf-8'))

                print(f"Firmware file uploaded: {file_path}")

            else:
                self.send_error(404, 'Not Found')

        except Exception as e:
            print(f"Upload error: {str(e)}")
            self.send_error(500, f'Upload failed: {str(e)}')

    def do_OPTIONS(self):
        """Handle CORS preflight requests"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def log_message(self, format, *args):
        """Override to customize logging"""
        print(f"HTTP: {format % args}")


class FirmwareUploadServer:
    """HTTP server for handling firmware uploads"""

    def __init__(self, port=8080, upload_dir='/shared/firmware_uploads'):
        self.port = port
        self.upload_dir = upload_dir
        self.server = None
        self.thread = None

    def start(self):
        """Start the HTTP server in a separate thread"""
        try:
            self.server = HTTPServer(('0.0.0.0', self.port), FirmwareUploadHandler)
            self.server.upload_dir = self.upload_dir  # Pass upload_dir to handler
            self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self.thread.start()
            print(f"HTTP upload server started on port {self.port}")
            print(f"Upload directory: {self.upload_dir}")
            return True
        except Exception as e:
            print(f"Failed to start HTTP upload server: {str(e)}")
            return False

    def stop(self):
        """Stop the HTTP server"""
        if self.server:
            self.server.shutdown()
            self.server.server_close()
        if self.thread:
            self.thread.join()
        print("HTTP upload server stopped")

    def is_running(self):
        """Check if the server is running"""
        return self.thread is not None and self.thread.is_alive()