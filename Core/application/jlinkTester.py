import sys
import time
import pylink
from pylink.errors import JLinkException
import struct
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QTextEdit, QVBoxLayout, QWidget
from PyQt5.QtCore import QThread, pyqtSignal

class JLinkWorker(QThread):
    output_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.jlink = None
        self.is_running = False

    def run(self):
        try:
            self.jlink = pylink.JLink()
            self.jlink.open()
            self.output_received.emit(f"Connected to J-Link (Serial Number: {self.jlink.serial_number})")
            
            # Connect to the target device
            self.jlink.connect('Cortex-M4', 50000)  # Adjust these parameters if needed
            self.output_received.emit("Connected to target device")
            
            # Check if RTT is enabled
            if not self.jlink.rtt_get_num_up_buffers():
                self.output_received.emit("RTT is not enabled on the target. Attempting to start RTT...")
                self.jlink.rtt_start()
            
            num_up_buffers = self.jlink.rtt_get_num_up_buffers()
            self.output_received.emit(f"Number of RTT up buffers: {num_up_buffers}")
            
            if num_up_buffers == 0:
                raise Exception("No RTT up buffers found. Make sure RTT is properly configured on the target.")
            
            self.is_running = True
            while self.is_running:
                start_time = time.time()
                data = self.jlink.rtt_read(0, 12)  # Read 12 bytes (3 floats) from RTT buffer 0
                if len(data) == 12:
                    iuDat, ivDat, iwDat = struct.unpack('fff', data)
                    output = f"iuDat: {iuDat:.4f}, ivDat: {ivDat:.4f}, iwDat: {iwDat:.4f}"
                    self.output_received.emit(output)
                elif len(data) > 0:
                    self.output_received.emit(f"Received partial data: {data}")
                else:
                    self.output_received.emit("No data received")
                
                # Add a small delay to prevent busy-waiting
                time.sleep(0.1)
                
                # Timeout after 5 seconds of no data
                if time.time() - start_time > 5:
                    self.output_received.emit("Timeout: No data received for 5 seconds")
                    
        except JLinkException as e:
            self.error_occurred.emit(f"J-Link error: {str(e)}")
        except Exception as e:
            self.error_occurred.emit(f"Error: {str(e)}")
        finally:
            if self.jlink:
                self.jlink.rtt_stop()
                self.jlink.close()
            self.is_running = False

    def stop(self):
        self.is_running = False
        self.wait()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("J-Link RTT Data Viewer")
        self.setGeometry(100, 100, 600, 400)

        layout = QVBoxLayout()

        self.connect_button = QPushButton("Connect to J-Link")
        self.connect_button.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_button)

        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        layout.addWidget(self.output_text)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.jlink_worker = JLinkWorker()
        self.jlink_worker.output_received.connect(self.update_output)
        self.jlink_worker.error_occurred.connect(self.show_error)

    def toggle_connection(self):
        if self.jlink_worker.isRunning():
            self.jlink_worker.stop()
            self.connect_button.setText("Connect to J-Link")
            self.output_text.append("Disconnected from J-Link")
        else:
            self.jlink_worker.start()
            self.connect_button.setText("Disconnect from J-Link")

    def update_output(self, text):
        self.output_text.append(text)

    def show_error(self, error_message):
        self.output_text.append(f"Error: {error_message}")
        self.connect_button.setText("Connect to J-Link")

    def closeEvent(self, event):
        self.jlink_worker.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())