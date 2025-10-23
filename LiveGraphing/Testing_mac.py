import sys, time, collections, os, re, csv, math, argparse
from pathlib import Path
from PySide6 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg
import serial, serial.tools.list_ports
try:
    import yaml
except ImportError:
    yaml = None

class SerialReader(QtCore.QThread):
    data_received = QtCore.Signal(str, float, float, float)  # port, ts, v_inst, v_filt
    disconnected = QtCore.Signal(str)  # port

    def __init__(self, port_name: str, baud: int = 115200, parent=None):
        super().__init__(parent)
        self._port_name = port_name
        self._baud = baud
        self._stop = False
        self._last_data_ts = 0.0

    def run(self):
        try:
            ser = serial.Serial(self._port_name, self._baud, timeout=1)
        except Exception as e:
            print(f"Failed to open {self._port_name}: {e}")
            try:
                self.disconnected.emit(self._port_name)
            except Exception:
                pass
            return
        with ser:
            while not self._stop:
                try:
                    line = ser.readline().decode(errors="ignore").strip()
                    if not line:
                        # If we haven't seen data in a while and the device disappeared, treat as disconnect
                        now = time.time()
                        if (now - self._last_data_ts) > 2.0:
                            try:
                                ports = [p.device for p in serial.tools.list_ports.comports() if "usbmodem" in p.device]
                                grp = _modem_group(self._port_name)
                                present = any(_modem_group(d) == grp for d in ports)
                                if not present:
                                    print(f"Port disappeared: {self._port_name}")
                                    break
                            except Exception:
                                pass
                        continue
                    parsed = self._extract_velocity(line)
                    if parsed is None:
                        continue
                    v_inst, v_filt = parsed
                    ts = time.time()
                    if v_filt is None:
                        v_filt = float('nan')
                    self.data_received.emit(self._port_name, ts, v_inst, v_filt)
                    self._last_data_ts = ts
                except ValueError:
                    continue
                except Exception as e:
                    print(f"Read error on {self._port_name}: {e}")
                    break
        try:
            self.disconnected.emit(self._port_name)
        except Exception:
            pass

    def stop(self):
        self._stop = True
        self.wait(500)

    @staticmethod
    def _extract_velocity(line: str):
        if not line:
            return None
        s = line.strip()
        if not s:
            return None
        # Direct float line
        try:
            return float(s), None
        except ValueError:
            pass

        v_inst = None
        v_filt = None

        # CSV format: timestamp,raw,net,force,v_inst,v_filt
        parts = [p.strip() for p in s.split(",")]
        if len(parts) >= 5:
            try:
                v_inst = float(parts[4])
            except ValueError:
                v_inst = None
            if len(parts) >= 6:
                try:
                    v_filt = float(parts[5])
                except ValueError:
                    v_filt = None

        # Pretty-print from STM32 (e.g., "v_inst=0.123 m/s | v_filt=0.456 m/s")
        if v_inst is None or v_filt is None:
            inst_match = re.search(r"v_inst\s*=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", s)
            filt_match = re.search(r"v_filt\s*=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", s)
            if inst_match and v_inst is None:
                try:
                    v_inst = float(inst_match.group(1))
                except ValueError:
                    v_inst = None
            if filt_match and v_filt is None:
                try:
                    v_filt = float(filt_match.group(1))
                except ValueError:
                    v_filt = None

        # Generic "key=value" tokens separated by pipes or commas
        if v_inst is None:
            for token in re.split(r"[|,]", s):
                if "v_inst" in token:
                    match = re.search(r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", token)
                    if match:
                        try:
                            v_inst = float(match.group(1))
                            break
                        except ValueError:
                            continue

        if v_filt is None:
            for token in re.split(r"[|,]", s):
                if "v_filt" in token:
                    match = re.search(r"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", token)
                    if match:
                        try:
                            v_filt = float(match.group(1))
                            break
                        except ValueError:
                            continue

        if v_inst is None:
            return None

        return v_inst, v_filt

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, devices, max_points=3000, refresh_ms=30, window_seconds=15, max_speed=10.0, min_speed=0.0, record_all=False):
        super().__init__()
        self.setWindowTitle("NUCLEO Live Plot")
        self.resize(900, 500)

        # Layout: top numeric readouts, bottom live plot
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)

        # Controls bar: logging toggle and status
        controls = QtWidgets.QWidget()
        controls_layout = QtWidgets.QHBoxLayout(controls)
        controls_layout.setContentsMargins(8, 8, 8, 0)
        controls_layout.setSpacing(12)
        self.btn_log = QtWidgets.QPushButton("Start Logging")
        self.lbl_log_status = QtWidgets.QLabel("Logging: OFF")
        self.lbl_log_status.setStyleSheet("color: #666;")
        controls_layout.addWidget(self.btn_log)
        controls_layout.addWidget(self.lbl_log_status)
        controls_layout.addStretch(1)
        main_layout.addWidget(controls)

        self.header = QtWidgets.QWidget()
        self.header_layout = QtWidgets.QHBoxLayout(self.header)
        self.header_layout.setContentsMargins(8, 8, 8, 8)
        self.header_layout.setSpacing(16)
        main_layout.addWidget(self.header)

        self.plot = pg.PlotWidget()
        main_layout.addWidget(self.plot, 1)
        self.plot.addLegend()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        # Control x-range manually for rolling window; fix y-range to avoid outliers
        try:
            self.plot.getViewBox().enableAutoRange(x=False, y=False)
            self.plot.setLabel('left', 'Speed', units='m/s')
            self.plot.setLabel('bottom', 'Time', units='s')
        except Exception:
            pass

        self.max_points = max_points
        self.buffers = {}
        self.curves = {}
        self.t0 = time.time()
        self.value_labels = {}
        self.port_to_reader = {}
        self.port_to_widget = {}
        self.window_seconds = float(window_seconds)
        self.max_speed = float(max_speed)
        self.min_speed = float(min_speed)
        self.port_to_name = {}
        self.is_student = {}
        self.logging_active = False
        self.log_rows = []
        self.log_by_port = {}
        self.log_start_ts = None
        self.record_all = bool(record_all)
        # Error line state
        self.err_lower = {}
        self.err_upper = {}
        self.err_fill = {}
        self.y_log = False
        self._log_mode_applied = False
        self.air_density = 1.204
        self.dp_error_pa = 0.62
        self.error_speed_floor = 0.2

        self.palette = [
            '#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
            '#9467bd', '#8c564b', '#e377c2', '#17becf', '#bcbd22'
        ]
        self.next_color_idx = 0
        self.readers = []
        self.known_ports = set()
        self.ground_truth_port = None
        self.ground_truth_group = None
        self.student_port = None

        for idx, dev in enumerate(devices):
            port = dev.get('port')
            name = dev.get('name') or port
            baud = int(dev.get('baud', 115200))
            if not port:
                continue
            grp = _modem_group(port)
            if self.ground_truth_port is None:
                self.ground_truth_port = port
                self.ground_truth_group = grp
                is_student = False
            else:
                is_student = (grp != self.ground_truth_group)
                if is_student and self.student_port is None:
                    self.student_port = port
            self.is_student[port] = is_student
            # buffers per port
            self.buffers[port] = {
                't': collections.deque(maxlen=self.max_points),
                'inst': collections.deque(maxlen=self.max_points),
                'filt': collections.deque(maxlen=self.max_points),
            }

            # curves per port
            if is_student:
                color_idx = self.next_color_idx % len(self.palette)
                inst_color = QtGui.QColor(self.palette[color_idx])
                filt_color = QtGui.QColor(self.palette[(color_idx + 1) % len(self.palette)])
                self.next_color_idx += 2
            else:
                inst_color = None
                filt_color = QtGui.QColor(self.palette[self.next_color_idx % len(self.palette)])
                self.next_color_idx += 1
            label = f"{name} ({port})" if name and name != port else port

            # numeric readout per device
            device_widget = QtWidgets.QWidget()
            device_layout = QtWidgets.QVBoxLayout(device_widget)
            device_layout.setContentsMargins(8, 4, 8, 4)
            title_label = QtWidgets.QLabel(label)
            title_label.setStyleSheet("color: #666; font-size: 12pt;")
            value_label = QtWidgets.QLabel("inst: —\nfilt: —" if is_student else "—")
            value_label.setStyleSheet("font-size: 24pt; font-weight: 600;")
            device_layout.addWidget(title_label)
            device_layout.addWidget(value_label)
            self.header_layout.addWidget(device_widget)
            self.value_labels[port] = value_label
            if is_student:
                inst_curve = self.plot.plot(pen=pg.mkPen(inst_color, width=2), name=f"{label} (inst)")
                filt_pen = pg.mkPen(filt_color, width=2, style=QtCore.Qt.PenStyle.DotLine)
                filt_curve = self.plot.plot(pen=filt_pen, name=f"{label} (filt)")
            else:
                inst_curve = None
                filt_curve = self.plot.plot(pen=pg.mkPen(filt_color, width=2), name=label)
            self.curves[port] = {
                'inst': inst_curve,
                'filt': filt_curve,
            }
            self.port_to_name[port] = name
            self.log_by_port.setdefault(port, [])

            # error lines for this device
            base_pen_source = inst_curve if inst_curve is not None else filt_curve
            base_pen = base_pen_source.opts['pen']
            qcolor = base_pen.color()
            dash_pen = pg.mkPen(qcolor, style=QtCore.Qt.PenStyle.DashLine)
            self.err_lower[port] = self.plot.plot(pen=dash_pen)
            self.err_upper[port] = self.plot.plot(pen=dash_pen)
            brush = pg.mkBrush(qcolor.red(), qcolor.green(), qcolor.blue(), 60)
            fbi = pg.FillBetweenItem(self.err_lower[port], self.err_upper[port], brush=brush)
            self.err_fill[port] = fbi
            self.plot.addItem(fbi)

            # serial reader per port
            reader = SerialReader(port, baud=baud)
            reader.data_received.connect(self.on_data)
            reader.disconnected.connect(self.on_disconnected)
            reader.start()
            self.readers.append(reader)
            self.known_ports.add(port)
            self.port_to_reader[port] = reader


        # Apply fixed y-range now that max_speed is known
        try:
            self.plot.setYRange(self.min_speed, self.max_speed, padding=0)
        except Exception:
            pass

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(int(refresh_ms))  # ~33 FPS by default

        # Hot-plug auto-detect: attach a second device when it appears
        self.desired_device_count = 2
        if len(self.curves) < self.desired_device_count:
            print("Waiting for additional /dev/tty.usbmodem* device to attach...")
        self.scan_timer = QtCore.QTimer(self)
        self.scan_timer.timeout.connect(self.scan_and_attach_new_devices)
        self.scan_timer.start(5000)

        # Hook up logging toggle
        self.btn_log.clicked.connect(self.toggle_logging)

        # Prepare output directory for log files
        try:
            self.log_dir = Path(__file__).resolve().parent.parent / "runs"
            self.log_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            print(f"[warn] Failed to ensure runs/ directory: {exc}. Falling back to current working directory.")
            self.log_dir = Path.cwd()

    @QtCore.Slot(str, float, float, float)
    def on_data(self, port, ts, value, value_filt):
        t_rel = ts - self.t0
        buf = self.buffers.get(port)
        if buf is None:
            return

        def clip(v):
            try:
                if v < self.min_speed:
                    return self.min_speed
                if v > self.max_speed:
                    return self.max_speed
            except Exception:
                pass
            return v

        inst_clipped = clip(value)
        if value_filt is None or math.isnan(value_filt):
            filt_clipped = float('nan')
        else:
            filt_clipped = clip(value_filt)

        tbuf = buf['t']
        inst_buf = buf['inst']
        filt_buf = buf['filt']
        tbuf.append(t_rel)
        inst_buf.append(inst_clipped)
        filt_buf.append(filt_clipped)

        lbl = self.value_labels.get(port)
        if lbl is not None:
            try:
                if self.is_student.get(port, False):
                    if math.isnan(filt_clipped):
                        lbl.setText(f"inst: {inst_clipped:.3f} m/s\nfilt: —")
                    else:
                        lbl.setText(f"inst: {inst_clipped:.3f} m/s\nfilt: {filt_clipped:.3f} m/s")
                else:
                    display_val = filt_clipped
                    if math.isnan(display_val):
                        display_val = inst_clipped
                    lbl.setText(f"{display_val:.3f} m/s")
            except Exception:
                pass

        # Logging rows: ISO time, epoch, t_rel, device_name, port, inst_raw, inst_clipped, filt_clipped, expected_error_pct
        try:
            if self.logging_active:
                iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(ts)) + f".{int((ts%1)*1000):03d}Z"
                rho = float(getattr(self, 'air_density', 1.204))
                dp = float(getattr(self, 'dp_error_pa', 0.62))
                vabs = abs(float(inst_clipped))
                is_gt = (_modem_group(port) == getattr(self, 'ground_truth_group', None))
                if is_gt and vabs > 1e-12:
                    dv_abs = dp / (rho * vabs)
                    err_pct = (dv_abs / vabs) * 100.0
                    err_pct_str = f"{err_pct:.3f}"
                else:
                    err_pct_str = ''
                row = [
                    iso,
                    f"{ts:.6f}",
                    f"{t_rel:.6f}",
                    self.port_to_name.get(port, port) or port,
                    port,
                    f"{float(value):.6f}",
                    f"{float(inst_clipped):.6f}",
                    '' if math.isnan(filt_clipped) else f"{float(filt_clipped):.6f}",
                    err_pct_str,
                ]
                self.log_rows.append(row)
                filt_entry = None if math.isnan(filt_clipped) else float(filt_clipped)
                self.log_by_port.setdefault(port, []).append((ts, float(inst_clipped), filt_entry))
        except Exception:
            pass

    def refresh(self):
        # Apply log mode if requested
        if not self._log_mode_applied and getattr(self, 'y_log', False):
            if self.min_speed <= 0:
                self.min_speed = 1e-6
            try:
                self.plot.setLogMode(y=True)
                self._log_mode_applied = True
            except Exception:
                pass

        latest_ts = None
        # Update curves and error lines; find latest timestamp
        for port, curve_dict in self.curves.items():
            buf = self.buffers.get(port)
            if not buf or not buf['t']:
                continue
            t_list = list(buf['t'])
            inst_list = list(buf['inst'])
            filt_list = list(buf['filt'])
            curve_inst = curve_dict.get('inst')
            curve_filt = curve_dict.get('filt')
            if curve_inst is not None:
                curve_inst.setData(t_list, inst_list)
            if curve_filt is not None:
                curve_filt.setData(t_list, filt_list)

            # dp-based error: dv = dp / (rho * max(|v|, floor))
            rho = float(getattr(self, 'air_density', 1.204))
            dp = float(getattr(self, 'dp_error_pa', 0.62))
            v_floor = float(getattr(self, 'error_speed_floor', 0.2))
            lower = []
            upper = []
            min_floor = self.min_speed if not getattr(self, 'y_log', False) else max(self.min_speed, 1e-6)
            for v in inst_list:
                denom = rho * max(abs(v), v_floor)
                dv = (dp / denom) if denom > 0 else 0.0
                lo = v - dv
                hi = v + dv
                if lo < min_floor:
                    lo = min_floor
                lower.append(lo)
                upper.append(hi)
            self.err_lower[port].setData(t_list, lower)
            self.err_upper[port].setData(t_list, upper)

            if latest_ts is None or t_list[-1] > latest_ts:
                latest_ts = t_list[-1]

        if latest_ts is not None:
            start = max(0.0, latest_ts - self.window_seconds)
            try:
                self.plot.setXRange(start, latest_ts, padding=0)
            except Exception:
                pass
        # Dynamic Y-axis: fit to current window max + 1, honor min_speed/log
        try:
            global_max = None
            if latest_ts is not None:
                t_cut = latest_ts - self.window_seconds
            else:
                t_cut = None
            for port in self.curves.keys():
                buf = self.buffers.get(port)
                if not buf or not buf['t']:
                    continue
                tbuf = buf['t']
                inst_vbuf = buf['inst']
                filt_vbuf = buf['filt']
                values = []
                local_max = None
                if t_cut is None:
                    values.extend(v for v in inst_vbuf if not math.isnan(v))
                    values.extend(v for v in filt_vbuf if not math.isnan(v))
                else:
                    for t, v in zip(tbuf, inst_vbuf):
                        if t >= t_cut and not math.isnan(v):
                            values.append(v)
                    for t, v in zip(tbuf, filt_vbuf):
                        if t >= t_cut and not math.isnan(v):
                            values.append(v)
                if values:
                    local_max = max(values)
                if local_max is not None:
                    global_max = local_max if global_max is None else (local_max if local_max > global_max else global_max)
            if global_max is not None:
                upper = global_max + 1.0
                lower = self.min_speed if not getattr(self, 'y_log', False) else max(self.min_speed, 1e-6)
                if upper <= lower:
                    upper = lower * 2.0
                self.plot.setYRange(lower, upper, padding=0)
        except Exception:
            pass

    def closeEvent(self, event):
        for r in self.readers:
            r.stop()
        super().closeEvent(event)

    def attach_device(self, port: str, name: str = 'student', baud: int = 115200):
        if port in self.curves:
            return
        grp = _modem_group(port)
        if self.ground_truth_port is None:
            self.ground_truth_port = port
            self.ground_truth_group = grp
            is_student = False
        else:
            is_student = (grp != self.ground_truth_group)
            if is_student:
                if self.student_port and self.student_port in self.curves and self.student_port != port:
                    self.remove_device(self.student_port)
                self.student_port = port
        self.is_student[port] = is_student
        # buffers
        self.buffers[port] = {
            't': collections.deque(maxlen=self.max_points),
            'inst': collections.deque(maxlen=self.max_points),
            'filt': collections.deque(maxlen=self.max_points),
        }
        # UI elements
        if is_student:
            color_idx = self.next_color_idx % len(self.palette)
            inst_color = QtGui.QColor(self.palette[color_idx])
            filt_color = QtGui.QColor(self.palette[(color_idx + 1) % len(self.palette)])
            self.next_color_idx += 2
        else:
            inst_color = None
            filt_color = QtGui.QColor(self.palette[self.next_color_idx % len(self.palette)])
            self.next_color_idx += 1
        label = f"{name} ({port})" if name and name != port else port
        device_widget = QtWidgets.QWidget()
        device_layout = QtWidgets.QVBoxLayout(device_widget)
        device_layout.setContentsMargins(8, 4, 8, 4)
        title_label = QtWidgets.QLabel(label)
        title_label.setStyleSheet("color: #666; font-size: 12pt;")
        value_label = QtWidgets.QLabel("inst: —\nfilt: —" if is_student else "—")
        value_label.setStyleSheet("font-size: 24pt; font-weight: 600;")
        device_layout.addWidget(title_label)
        device_layout.addWidget(value_label)
        self.header_layout.addWidget(device_widget)
        self.value_labels[port] = value_label
        self.port_to_widget[port] = device_widget
        if is_student:
            inst_curve = self.plot.plot(pen=pg.mkPen(inst_color, width=2), name=f"{label} (inst)")
            filt_pen = pg.mkPen(filt_color, width=2, style=QtCore.Qt.PenStyle.DotLine)
            filt_curve = self.plot.plot(pen=filt_pen, name=f"{label} (filt)")
        else:
            inst_curve = None
            filt_curve = self.plot.plot(pen=pg.mkPen(filt_color, width=2), name=label)
        self.curves[port] = {
            'inst': inst_curve,
            'filt': filt_curve,
        }
        self.port_to_name[port] = name
        self.log_by_port.setdefault(port, [])
        # Error lines
        base_pen_source = inst_curve if inst_curve is not None else filt_curve
        base_pen = base_pen_source.opts['pen']
        qcolor = base_pen.color()
        dash_pen = pg.mkPen(qcolor, style=QtCore.Qt.PenStyle.DashLine)
        self.err_lower[port] = self.plot.plot(pen=dash_pen)
        self.err_upper[port] = self.plot.plot(pen=dash_pen)
        brush = pg.mkBrush(qcolor.red(), qcolor.green(), qcolor.blue(), 60)
        fbi = pg.FillBetweenItem(self.err_lower[port], self.err_upper[port], brush=brush)
        self.err_fill[port] = fbi
        self.plot.addItem(fbi)
        # Reader
        reader = SerialReader(port, baud=baud)
        reader.data_received.connect(self.on_data)
        reader.disconnected.connect(self.on_disconnected)
        reader.start()
        self.readers.append(reader)
        self.known_ports.add(port)
        self.port_to_reader[port] = reader
        print(f"Attached new device: {label} @ {baud}")

    @QtCore.Slot()
    def scan_and_attach_new_devices(self):
        try:
            if len(self.curves) >= self.desired_device_count:
                return
            known_groups = {_modem_group(p) for p in self.known_ports}
            available = [p for p in _list_available_device_paths() if p not in self.known_ports and _modem_group(p) not in known_groups]
            if available:
                # Attach the first new usbmodem device we see
                self.attach_device(available[0], name='student', baud=115200)
        except Exception as e:
            # Avoid crashing the UI if scan fails
            print(f"Scan error: {e}")

    @QtCore.Slot(str)
    def on_disconnected(self, port: str):
        try:
            if port not in self.curves:
                return
            print(f"Device disconnected: {port}")
            self.remove_device(port)
            if self.student_port == port:
                self.student_port = None
        except Exception as e:
            print(f"Error handling disconnect for {port}: {e}")

    def remove_device(self, port: str):
        # Stop and remove reader
        reader = self.port_to_reader.pop(port, None)
        if reader is not None:
            try:
                reader.stop()
            except Exception:
                pass
        # Remove plot curves
        curve_dict = self.curves.pop(port, None)
        if curve_dict is not None:
            for item in curve_dict.values():
                if item is None:
                    continue
                try:
                    self.plot.removeItem(item)
                except Exception:
                    pass
        # Remove error items
        lower = self.err_lower.pop(port, None)
        upper = self.err_upper.pop(port, None)
        fill = self.err_fill.pop(port, None)
        for item in (lower, upper, fill):
            if item is not None:
                try:
                    self.plot.removeItem(item)
                except Exception:
                    pass
        # Remove header widget
        widget = self.port_to_widget.pop(port, None)
        if widget is not None:
            try:
                widget.setParent(None)
                widget.deleteLater()
            except Exception:
                pass
        # Remove buffers and labels
        self.buffers.pop(port, None)
        self.value_labels.pop(port, None)
        self.is_student.pop(port, None)
        if port in self.known_ports:
            self.known_ports.remove(port)
        self.port_to_name.pop(port, None)

    @QtCore.Slot()
    def toggle_logging(self):
        if not self.logging_active:
            # Start logging
            self.logging_active = True
            self.log_rows = []
            self.log_by_port = {p: [] for p in self.curves.keys()}
            self.log_start_ts = time.time()
            self.btn_log.setText("Stop && Save")
            self.lbl_log_status.setText("Logging: ON")
            self.lbl_log_status.setStyleSheet("color: #0a0;")
            self.lbl_log_status.setToolTip("")
        else:
            # Stop logging and prompt to save
            self.logging_active = False
            self.btn_log.setText("Start Logging")
            self.lbl_log_status.setStyleSheet("color: #666;")
            if not self.log_rows:
                self.lbl_log_status.setText("Logging: OFF")
                self.log_start_ts = None
                return
            try:
                ports = list(self.log_by_port.keys())
                aligned = len(ports) >= 2 and bool(getattr(self, 'log_rate_hz', None))
                suggested_path = self._next_log_path(aligned)
                path, _ = QtWidgets.QFileDialog.getSaveFileName(
                    self,
                    "Save CSV",
                    str(suggested_path),
                    "CSV Files (*.csv)",
                )
                if not path:
                    self.lbl_log_status.setText("Logging: OFF")
                    self.lbl_log_status.setToolTip("")
                    return
                chosen_name = Path(path).name
                if not chosen_name.lower().endswith('.csv'):
                    chosen_name += '.csv'
                target_path = self.log_dir / chosen_name
                self._save_log_csv(str(target_path))
                self.lbl_log_status.setText(f"Saved: {target_path.name}")
                self.lbl_log_status.setToolTip(str(target_path))
                print(f"Saved log to {target_path}")
            except Exception as e:
                self.lbl_log_status.setText("Save failed")
                self.lbl_log_status.setToolTip(str(e))
                print(f"Failed to save CSV: {e}")
            finally:
                self.log_start_ts = None

    def _next_log_path(self, aligned: bool) -> Path:
        ts = self.log_start_ts or time.time()
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(ts))
        suffix = "-aligned" if aligned else ""
        base_name = f"log-{stamp}{suffix}.csv"
        candidate = self.log_dir / base_name
        counter = 1
        while candidate.exists():
            candidate = self.log_dir / f"log-{stamp}{suffix}-{counter:02d}.csv"
            counter += 1
        return candidate

    def _save_log_csv(self, filepath: str):
        # If we have two ports, produce fixed-rate samples aligned to a uniform grid
        ports = list(self.log_by_port.keys())
        if len(ports) >= 2 and getattr(self, 'log_rate_hz', None):
            gt_port = None
            student_port = None
            for p in ports:
                if _modem_group(p) == self.ground_truth_group:
                    gt_port = p
                else:
                    student_port = p if student_port is None else student_port
            if gt_port is None and ports:
                gt_port = ports[0]
            if student_port is None and len(ports) > 1:
                student_port = [p for p in ports if p != gt_port][0]

            gt_series = sorted(self.log_by_port.get(gt_port, []))
            st_series = sorted(self.log_by_port.get(student_port, []))
            if gt_series and st_series:
                t0 = min(gt_series[0][0], st_series[0][0])
                t1 = max(gt_series[-1][0], st_series[-1][0])
                step = 1.0 / float(self.log_rate_hz)
                tol = float(getattr(self, 'align_tolerance_ms', 40)) / 1000.0
                # Build grid from t0 to t1 inclusive
                num_steps = int((t1 - t0) / step) + 1
                grid = [t0 + i * step for i in range(num_steps)]

                def sample_nearest(series, t_target, value_index=1):
                    # binary search nearest
                    lo, hi = 0, len(series) - 1
                    best_idx = 0
                    best_dt = float('inf')
                    while lo <= hi:
                        mid = (lo + hi) // 2
                        tm = series[mid][0]
                        dt = abs(tm - t_target)
                        if dt < best_dt:
                            best_dt = dt
                            best_idx = mid
                        if tm < t_target:
                            lo = mid + 1
                        elif tm > t_target:
                            hi = mid - 1
                        else:
                            break
                    if best_dt <= tol:
                        val = series[best_idx][value_index] if value_index < len(series[best_idx]) else None
                        return '' if val is None else val
                    return ''  # empty if no sample within tolerance

                aligned_rows = []
                record_all = bool(getattr(self, 'record_all', False))
                for tg in grid:
                    iso = time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(tg)) + f".{int((tg%1)*1000):03d}Z"
                    v_gt = sample_nearest(gt_series, tg)
                    v_st = sample_nearest(st_series, tg)
                    v_st_filt = sample_nearest(st_series, tg, value_index=2) if record_all else ''
                    def pct(v):
                        if v == '':
                            return ''
                        try:
                            v_f = float(v)
                            vabs = abs(v_f)
                            if vabs <= 1e-12:
                                return ''
                            rho = float(getattr(self, 'air_density', 1.204))
                            dp = float(getattr(self, 'dp_error_pa', 0.62))
                            dv_abs = dp / (rho * vabs)
                            return f"{(dv_abs / vabs) * 100.0:.3f}"
                        except Exception:
                            return ''
                    row = [
                        iso,
                        f"{tg:.6f}",
                        f"{v_gt}" if v_gt == '' else f"{float(v_gt):.6f}",
                        f"{v_st}" if v_st == '' else f"{float(v_st):.6f}",
                    ]
                    if record_all:
                        row.append(f"{v_st_filt}" if v_st_filt == '' else f"{float(v_st_filt):.6f}")
                    row.extend([
                        pct(v_gt),
                        '',  # student error unknown
                    ])
                    aligned_rows.append(row)

                header = ['timestamp_iso', 'timestamp_epoch', 'ground_truth_mps', 'student_mps']
                if record_all:
                    header.append('student_mps_filtered')
                header.extend(['ground_truth_err_pct', 'student_err_pct'])
                try:
                    with open(filepath, 'w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(header)
                        writer.writerows(aligned_rows)
                    print(f"Saved fixed-rate aligned log to {filepath} ({len(aligned_rows)} rows @ {self.log_rate_hz} Hz)")
                    return
                except Exception as e:
                    print(f"Error writing fixed-rate CSV {filepath}: {e}")
                    # Fall through to raw format

        # Raw per-sample rows if only one port or alignment failed
        header = [
            'timestamp_iso', 'timestamp_epoch', 't_rel_s',
            'device_name', 'port', 'value_inst_raw_mps', 'value_inst_clipped_mps',
            'value_filtered_mps', 'expected_error_pct'
        ]
        try:
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(self.log_rows)
            print(f"Saved log to {filepath} ({len(self.log_rows)} rows)")
        except Exception as e:
            print(f"Error writing CSV {filepath}: {e}")

def list_ports():
    print("Available serial ports:")
    for p in serial.tools.list_ports.comports():
        print(f" - {p.device}: {p.manufacturer} {p.product} SN={p.serial_number}")

def _modem_group(path: str):
    m = re.search(r"usbmodem(.*)$", path)
    return m.group(1) if m else path

def _list_available_device_paths():
    ports = list(serial.tools.list_ports.comports())
    # Consider both tty.usbmodem* and cu.usbmodem*
    groups = {}
    for p in ports:
        dev = p.device
        if "usbmodem" not in dev:
            continue
        g = _modem_group(dev)
        entry = groups.setdefault(g, {"tty": None, "cu": None})
        if "/tty." in dev:
            entry["tty"] = dev
        elif "/cu." in dev:
            entry["cu"] = dev
    # Prefer tty if present, otherwise cu
    preferred = []
    for g, entry in groups.items():
        preferred.append(entry["tty"] or entry["cu"])
    preferred = [p for p in preferred if p]
    preferred.sort()
    return preferred

def _resolve_devices_from_config(config_devices):
    # Normalize and auto-detect missing ports
    all_ports = _list_available_device_paths()
    specified_ports = [d.get('port') for d in config_devices if d.get('port')]
    specified_groups = {_modem_group(p) for p in specified_ports}
    remaining = [p for p in all_ports if p not in specified_ports and _modem_group(p) not in specified_groups]

    resolved = []
    for d in (config_devices or []):
        name = d.get('name') or d.get('port') or 'device'
        baud = int(d.get('baud', 115200))
        port = d.get('port')
        if not port:
            port = remaining.pop(0) if remaining else None
        if port:
            resolved.append({'name': name, 'port': port, 'baud': baud})

    # If only one device configured (e.g., ground truth) and another port exists, add it as student
    if len(resolved) < 2 and len(all_ports) >= 2:
        used = {r['port'] for r in resolved}
        used_groups = {_modem_group(p) for p in used}
        extras = [p for p in all_ports if p not in used and _modem_group(p) not in used_groups]
        if extras:
            resolved.append({'name': 'student', 'port': extras[0], 'baud': 115200})

    return resolved

if __name__ == "__main__":
    if yaml is None:
        print("PyYAML is required. Install with: python3 -m pip install pyyaml")
        sys.exit(1)

    parser = argparse.ArgumentParser(description="NUCLEO live plotter")
    parser.add_argument("--config", default=None, help="Optional path to config.yaml")
    parser.add_argument("--record-all", action="store_true", help="Include both instant and filtered student values in aligned CSV logs")
    args, qt_args = parser.parse_known_args()
    qt_argv = [sys.argv[0], *qt_args]

    config_path = args.config or os.path.join(os.path.dirname(__file__), "config.yaml")
    if not os.path.exists(config_path):
        print(f"Config not found: {config_path}")
        list_ports()
        sys.exit(1)

    try:
        with open(config_path, "r") as f:
            config = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"Failed to read config.yaml: {e}")
        sys.exit(1)

    devices = config.get("devices", []) or []
    devices = _resolve_devices_from_config(devices)
    print("Resolved devices:")
    for d in devices:
        try:
            print(f" - {d['name']}: {d['port']} @ {d['baud']}")
        except Exception:
            pass
    ui_cfg = config.get("ui", {}) or {}
    max_points = int(ui_cfg.get("max_points", 3000))
    refresh_ms = int(ui_cfg.get("refresh_ms", 30))
    window_seconds = float(ui_cfg.get("window_seconds", 15))
    max_speed = float(ui_cfg.get("max_speed", 10))
    min_speed = float(ui_cfg.get("min_speed", 0))
    log_rate_hz = float(ui_cfg.get("log_rate_hz", 20))
    align_tolerance_ms = float(ui_cfg.get("align_tolerance_ms", 40))
    y_log = bool(ui_cfg.get("y_log", False))
    air_density = float(ui_cfg.get("air_density", 1.204))
    dp_error_pa = float(ui_cfg.get("dp_error_pa", 0.62))
    error_speed_floor = float(ui_cfg.get("error_speed_floor", 0.2))

    if not devices:
        print("No devices configured in config.yaml under 'devices'.")
        list_ports()
        sys.exit(1)

    app = QtWidgets.QApplication(qt_argv)
    w = MainWindow(devices, max_points=max_points, refresh_ms=refresh_ms, window_seconds=window_seconds, max_speed=max_speed, min_speed=min_speed, record_all=args.record_all)
    w.log_rate_hz = log_rate_hz
    w.align_tolerance_ms = align_tolerance_ms
    w.y_log = y_log
    w.air_density = air_density
    w.dp_error_pa = dp_error_pa
    w.error_speed_floor = error_speed_floor
    w.show()
    sys.exit(app.exec())
