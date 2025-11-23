import os
import subprocess
import sys
from functools import partial

from python_qt_binding import QtCore, QtWidgets
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from std_srvs.srv import Trigger


class CalibGui(QtWidgets.QWidget):
  def __init__(self, ros_if, parent=None):
    super().__init__(parent)
    self.ros_if = ros_if
    self.calib_process = None
    self.setWindowTitle("Camera Calibration (Eye-on-Hand)")
    self._build_ui()

  def _build_ui(self):
    layout = QtWidgets.QVBoxLayout()

    form = QtWidgets.QFormLayout()
    self.base_frame = QtWidgets.QLineEdit("base_link")
    self.gripper_frame = QtWidgets.QLineEdit("link6")
    self.camera_frame = QtWidgets.QLineEdit("camera_link")
    self.target_frame = QtWidgets.QLineEdit("tag_frame")
    self.output_path = QtWidgets.QLineEdit(os.path.expanduser("~/axab_calibration.yaml"))
    self.min_samples = QtWidgets.QSpinBox()
    self.min_samples.setMinimum(3)
    self.min_samples.setMaximum(1000)
    self.min_samples.setValue(8)

    form.addRow("Base frame", self.base_frame)
    form.addRow("Gripper frame", self.gripper_frame)
    form.addRow("Camera frame", self.camera_frame)
    form.addRow("Target frame", self.target_frame)
    form.addRow("Output YAML path", self.output_path)
    form.addRow("Minimum samples", self.min_samples)
    layout.addLayout(form)

    button_layout = QtWidgets.QHBoxLayout()
    self.start_btn = QtWidgets.QPushButton("Start Calibrator")
    self.stop_btn = QtWidgets.QPushButton("Stop Calibrator")
    self.stop_btn.setEnabled(False)
    button_layout.addWidget(self.start_btn)
    button_layout.addWidget(self.stop_btn)
    layout.addLayout(button_layout)

    action_layout = QtWidgets.QHBoxLayout()
    self.add_btn = QtWidgets.QPushButton("Add Sample")
    self.compute_btn = QtWidgets.QPushButton("Compute")
    self.save_btn = QtWidgets.QPushButton("Save YAML")
    self.reset_btn = QtWidgets.QPushButton("Reset Samples")
    action_layout.addWidget(self.add_btn)
    action_layout.addWidget(self.compute_btn)
    action_layout.addWidget(self.save_btn)
    action_layout.addWidget(self.reset_btn)
    layout.addLayout(action_layout)

    self.status = QtWidgets.QPlainTextEdit()
    self.status.setReadOnly(True)
    layout.addWidget(self.status)

    self.setLayout(layout)

    self.start_btn.clicked.connect(self.start_calibrator)
    self.stop_btn.clicked.connect(self.stop_calibrator)
    self.add_btn.clicked.connect(partial(self._call_trigger, "add_sample"))
    self.compute_btn.clicked.connect(self.compute_only)
    self.save_btn.clicked.connect(self.save_yaml)
    self.reset_btn.clicked.connect(partial(self._call_trigger, "reset_samples"))

  def log(self, text):
    self.status.appendPlainText(text)

  def start_calibrator(self):
    if self.calib_process is not None:
      self.log("Calibrator already running.")
      return

    cmd = [
      "ros2",
      "run",
      "camera_calibration",
      "eye_on_hand_calibrator",
      "--ros-args",
      "-p",
      f"base_frame:={self.base_frame.text()}",
      "-p",
      f"gripper_frame:={self.gripper_frame.text()}",
      "-p",
      f"camera_frame:={self.camera_frame.text()}",
      "-p",
      f"target_frame:={self.target_frame.text()}",
      "-p",
      f"output_path:={self.output_path.text()}",
      "-p",
      f"min_samples:={self.min_samples.value()}",
    ]
    self.log("Starting calibrator...")
    try:
      self.calib_process = subprocess.Popen(cmd)
      self.log("Calibrator started.")
      self.start_btn.setEnabled(False)
      self.stop_btn.setEnabled(True)
    except Exception as exc:
      self.log(f"Failed to start calibrator: {exc}")
      self.calib_process = None

  def stop_calibrator(self):
    if self.calib_process is None:
      self.log("Calibrator is not running.")
      return
    self.log("Stopping calibrator...")
    self.calib_process.terminate()
    try:
      self.calib_process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
      self.calib_process.kill()
    self.calib_process = None
    self.start_btn.setEnabled(True)
    self.stop_btn.setEnabled(False)
    self.log("Calibrator stopped.")

  def _call_trigger(self, service_name):
    def done_callback(future):
      try:
        res = future.result()
        prefix = "OK" if res.success else "ERROR"
        self.log(f"[{service_name}] {prefix}: {res.message}")
      except Exception as exc:
        self.log(f"[{service_name}] call failed: {exc}")

    client = self.ros_if.get_client(service_name)
    if client is None:
      self.log(f"Service /{service_name} unavailable. Ensure calibrator is running.")
      return

    future = client.call_async(Trigger.Request())
    future.add_done_callback(done_callback)

  def compute_only(self):
    self._call_trigger("compute_calibration")

  def save_yaml(self):
    self._call_trigger("save_calibration")


class RosInterface(Node):
  def __init__(self):
    super().__init__("camera_calibration_gui")
    self._clients = {}

  def get_client(self, service_name):
    if service_name in self._clients:
      return self._clients[service_name]
    client = self.create_client(Trigger, service_name, qos_profile=qos_profile_services_default)
    if not client.wait_for_service(timeout_sec=0.5):
      self.get_logger().warn(f"Service {service_name} not available")
      return None
    self._clients[service_name] = client
    return client


def main(args=None):
  rclpy.init(args=args)
  ros_if = RosInterface()

  app = QtWidgets.QApplication(sys.argv)
  widget = CalibGui(ros_if)
  widget.resize(520, 320)
  widget.show()

  # Periodically spin rclpy to service clients
  timer = QtCore.QTimer()
  timer.timeout.connect(lambda: rclpy.spin_once(ros_if, timeout_sec=0.01))
  timer.start(10)

  ret = app.exec_()

  if widget.calib_process is not None:
    widget.stop_calibrator()

  rclpy.shutdown()
  sys.exit(ret)


if __name__ == "__main__":
  main()
