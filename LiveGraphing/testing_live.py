#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
testing_live_cfit.py
- Student-only friendly live plot (GT optional)
- Fits aerodynamic drag constant using F ≈ 0.5*rho*C*A*v^2 + b
- Outputs ready-to-paste #defines for main.c

Ways to provide Ground Truth (GT):
  1) Live serial: --gt-port /dev/tty.usbmodemXXXX
     Accepts either a single float per line (v m/s) or "t_ms,v".
  2) Offline CSV: --gt-csv path.csv  with columns ["t_ms","v"] or a single "v" column.

If you don’t have GT, you can still plot Student; the C fit needs GT at some point.

Usage examples:
  # Student only live plot
  python3 testing_live_cfit.py --student-port /dev/tty.usbmodem11403

  # Live with GT serial and known plate size (meters)
  python3 testing_live_cfit.py --student-port /dev/tty.usbmodem11403 \
      --gt-port /dev/tty.usbmodem141303 --plate-w 0.09 --plate-h 0.093 --rho 1.204

  # Student live + later load GT CSV for fitting (press 'g' to fit after loading)
  python3 testing_live_cfit.py --student-port /dev/tty.usbmodem11403 --gt-csv gt_log.csv
"""

import argparse, os, sys, time, threading, queue, csv, math, json
from dataclasses import dataclass
from datetime import datetime

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ----------------------------- parsing -----------------------------

def parse_student(line):
    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 6:  # tolerate chatter
        return None
    try:
        t_ms   = int(parts[0])
        raw    = int(float(parts[1]))
        net    = int(float(parts[2]))
        fN     = float(parts[3])        # <- we use this for fitting
        v_inst = float(parts[4])
        v_filt = float(parts[5])
        return dict(t_ms=t_ms, raw=raw, net=net, fN=fN, v_inst=v_inst, v_filt=v_filt)
    except:
        return None

def parse_gt_line(line):
    # either a single float (v) or "t_ms,v"
    try:
        v = float(line.strip())
        return dict(t_ms=None, v=v)
    except:
        pass
    parts = [p.strip() for p in line.split(",")]
    if len(parts) >= 2:
        try:
            return dict(t_ms=int(parts[0]), v=float(parts[1]))
        except:
            return None
    return None

# ----------------------------- IO -----------------------------

def open_serial(port, baud):
    if not port:
        return None
    try:
        return serial.Serial(port, baudrate=baud, timeout=0.05)
    except Exception as e:
        print(f"[WARN] Could not open {port}: {e}")
        return None

def tail_serial(ser, q, kind):
    buf = b""
    while True:
        try:
            chunk = ser.read(4096)
        except Exception:
            break
        if not chunk:
            time.sleep(0.01)
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            s = line.decode(errors="ignore").strip()
            if not s:
                continue
            if kind == "ST":
                rec = parse_student(s)
                if rec: q.put(("ST", rec))
            else:
                rec = parse_gt_line(s)
                if rec: q.put(("GT", rec))

def load_gt_csv(path):
    out = []
    with open(path, "r") as f:
        r = csv.DictReader(f)
        heads = [h.strip().lower() for h in r.fieldnames] if r.fieldnames else []
        # support ["t_ms","v"] or ["v"]
        if "v" in heads:
            for row in r:
                v = float(row["v"])
                t = int(row["t_ms"]) if "t_ms" in heads and row.get("t_ms","").strip() else None
                out.append(dict(t_ms=t, v=v))
        else:
            # try: first column named ? single value rows
            f.seek(0)
            rr = csv.reader(f)
            for row in rr:
                if not row: continue
                try:
                    out.append(dict(t_ms=None, v=float(row[0])))
                except:
                    continue
    return out

# ----------------------------- fitting -----------------------------

@dataclass
class FitConfig:
    rho: float      # air density
    plate_w: float  # meters (optional)
    plate_h: float  # meters (optional)

    @property
    def area(self):
        if self.plate_w and self.plate_h:
            return self.plate_w * self.plate_h
        return None

def fit_drag(F_list, v_list):
    """
    Fit F = K*v^2 + b via ordinary least squares.
    Returns (K,b,R2,n)
    """
    import numpy as np
    v2 = np.array([vi*vi for vi in v_list], dtype=float)
    F  = np.array(F_list, dtype=float)
    X  = np.column_stack([v2, np.ones_like(v2)])
    beta, *_ = np.linalg.lstsq(X, F, rcond=None)
    K, b = beta[0], beta[1]
    # R^2
    Fhat = X @ beta
    ss_res = float(((F - Fhat)**2).sum())
    ss_tot = float(((F - F.mean())**2).sum()) if len(F)>1 else 0.0
    R2 = 1.0 - ss_res/ss_tot if ss_tot>0 else 0.0
    return K, b, R2, len(F)

def derive_params(K, fitcfg: FitConfig):
    """
    If area known: K = 0.5*rho*C*A  => C = 2K/(rho*A)
    If area unknown: report DRAG_CA = C*A = 2K/rho (m^2)
    """
    rho = fitcfg.rho
    A   = fitcfg.area
    if A and A > 0:
        C  = 2.0*K/(rho*A)
        return dict(C_PLATE=float(C), DRAG_CA=float(C*A))
    else:
        CA = 2.0*K/rho
        return dict(DRAG_CA=float(CA))

# ----------------------------- plot -----------------------------

class LivePlot:
    def __init__(self, window_s=15, y_max=30):
        self.t0 = time.time()
        self.window_s = window_s
        self.st_t, self.st_vf, self.gt_t, self.gt_v = [], [], [], []

        import matplotlib.pyplot as plt
        self.fig, self.ax = plt.subplots(figsize=(9,5))
        (self.l_st,) = self.ax.plot([], [], label="Student v_filt")
        (self.l_gt,) = self.ax.plot([], [], label="GT v")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (m/s)")
        self.ax.set_ylim(-0.5, y_max)
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper right")

    def add_student(self, t_ms, v_filt):
        t = (t_ms/1000.0) if t_ms is not None else (time.time() - self.t0)
        self.st_t.append(t); self.st_vf.append(v_filt)
        self._trim()

    def add_gt(self, t_ms, v):
        t = (t_ms/1000.0) if t_ms is not None else (time.time() - self.t0)
        self.gt_t.append(t); self.gt_v.append(v)
        self._trim()

    def _trim(self):
        for (tlist, *_) in [(self.st_t,), (self.gt_t,)]:
            if not tlist: continue
            tmax = tlist[-1]
            while tlist and (tmax - tlist[0]) > self.window_s:
                if tlist is self.st_t:
                    self.st_t.pop(0); self.st_vf.pop(0)
                else:
                    self.gt_t.pop(0); self.gt_v.pop(0)

    def update(self, _frame):
        self.l_st.set_data(self.st_t, self.st_vf)
        self.l_gt.set_data(self.gt_t, self.gt_v)
        right = self.st_t[-1] if self.st_t else self.gt_t[-1] if self.gt_t else self.window_s
        left  = max(0.0, right - self.window_s)
        self.ax.set_xlim(left, right)
        return self.l_st, self.l_gt

# ----------------------------- main -----------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--student-port", required=True)
    ap.add_argument("--student-baud", type=int, default=115200)
    ap.add_argument("--gt-port", default=None)
    ap.add_argument("--gt-baud", type=int, default=115200)
    ap.add_argument("--gt-csv", default=None, help="Load GT CSV for fitting (columns: v or t_ms,v)")
    ap.add_argument("--rho", type=float, default=1.204, help="Air density (kg/m^3), default ~20°C sea-level")
    ap.add_argument("--plate-w", type=float, default=0.0, help="Plate width (m)")
    ap.add_argument("--plate-h", type=float, default=0.0, help="Plate height (m)")
    ap.add_argument("--logdir", default="runs")
    ap.add_argument("--ymax", type=float, default=30.0)
    args = ap.parse_args()

    os.makedirs(args.logdir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_csv = os.path.join(args.logdir, f"wind_{ts}.csv")
    fit_json = os.path.join(args.logdir, f"fit_{ts}.json")
    header_h = os.path.join(args.logdir, f"drag_{ts}.h")

    # open serials
    ser_st = open_serial(args.student_port, args.student_baud)
    if not ser_st:
        print("[ERR] Cannot open Student port.")
        sys.exit(2)
    ser_gt = open_serial(args.gt_port, args.gt_baud) if args.gt_port else None

    # offline GT
    gt_offline = []
    if args.gt_csv and os.path.exists(args.gt_csv):
        gt_offline = load_gt_csv(args.gt_csv)
        print(f"[GT] Loaded offline GT CSV: {len(gt_offline)} rows")

    # queues + threads
    q = queue.Queue()
    threading.Thread(target=tail_serial, args=(ser_st, q, "ST"), daemon=True).start()
    if ser_gt:
        threading.Thread(target=tail_serial, args=(ser_gt, q, "GT"), daemon=True).start()

    # logger
    f = open(log_csv, "w", newline="")
    cw = csv.writer(f)
    cw.writerow(["src","t_ms","force_N","v_inst","v_filt","v_gt"])

    # buffers for fit
    student_F = []   # force_N
    gt_V = []        # v (m/s), from live or offline
    # live plot
    lp = LivePlot(y_max=args.ymax)
    ani = animation.FuncAnimation(lp.fig, lp.update, interval=40)
    print("\nKeys:  g=fit C   o=load GT CSV   q=quit\n")

    keyq = queue.Queue()
    def on_key(e):
        k = (e.key or "").lower()
        if k in {"g","o","q"}: keyq.put(k)
    lp.fig.canvas.mpl_connect("key_press_event", on_key)

    fitcfg = FitConfig(rho=args.rho, plate_w=args.plate_w, plate_h=args.plate_h)

    # merge function: take last N aligned samples (simple min(len))
    def run_fit():
        import numpy as np
        n = min(len(student_F), len(gt_V))
        if n < 50:
            print(f"[FIT] Not enough paired samples (have {n}, need >= 50).")
            return
        F = student_F[-n:]
        V = gt_V[-n:]
        K,b,R2,N = fit_drag(F, V)
        derived = derive_params(K, fitcfg)
        print("\n===== Drag fit (F = K*v^2 + b) =====")
        print(f"  K (N / (m/s)^2): {K:.6e}")
        print(f"  b (N)          : {b:.6e}")
        print(f"  R^2            : {R2:.4f}   (N={N})")
        if "C_PLATE" in derived:
            print(f"  C_PLATE        : {derived['C_PLATE']:.6f}   (dimensionless)")
            print(f"  DRAG_CA (m^2)  : {derived['DRAG_CA']:.6e}")
        else:
            print(f"  DRAG_CA (m^2)  : {derived['DRAG_CA']:.6e}   (use this if firmware multiplies by 0.5*rho)")
        print("=====================================\n")

        # emit header snippet
        with open(header_h, "w") as fh:
            fh.write("// Auto-generated drag parameters from testing_live_cfit.py\n")
            fh.write("// Model: F = 0.5*rho*C*A*v^2 + b\n")
            fh.write(f"#define AIR_DENSITY_KGPM3   {fitcfg.rho:.6f}f\n")
            if fitcfg.area:
                fh.write(f"#define PLATE_AREA_M2       {fitcfg.area:.9f}f\n")
            fh.write(f"#define DRAG_K_N_PER_V2     {K:.9e}f  // slope K of F=K*v^2+b\n")
            fh.write(f"#define DRAG_BIAS_N         {b:.9e}f\n")
            if "C_PLATE" in derived:
                fh.write(f"#define C_PLATE             {derived['C_PLATE']:.9f}f\n")
            fh.write(f"#define DRAG_CA_M2          {derived['DRAG_CA']:.9e}f\n")
        with open(fit_json, "w") as jf:
            json.dump(dict(K=K,b=b,R2=R2,N=N,**derived, rho=fitcfg.rho, area=fitcfg.area), jf, indent=2)
        print(f"[OUT] Wrote {header_h}")
        print(f"[OUT] Wrote {fit_json}")

    # main loop
    try:
        gt_idx = 0
        while True:
            # pump queues
            try:
                src, rec = q.get_nowait()
            except queue.Empty:
                src = None

            if src == "ST":
                cw.writerow(["ST", rec["t_ms"], rec["fN"], rec["v_inst"], rec["v_filt"], ""])
                f.flush()
                student_F.append(rec["fN"])
                lp.add_student(rec["t_ms"], rec["v_filt"])
            elif src == "GT":
                cw.writerow(["GT", rec["t_ms"], "", "", "", rec["v"]])
                f.flush()
                gt_V.append(rec["v"])
                lp.add_gt(rec["t_ms"], rec["v"])

            # feed offline GT gradually (to keep roughly time-aligned)
            if gt_offline and (time.time() % 0.02 < 0.001):  # ~50 Hz trickle
                if gt_idx < len(gt_offline):
                    v = gt_offline[gt_idx]["v"]
                    cw.writerow(["GT", gt_offline[gt_idx]["t_ms"], "", "", "", v])
                    f.flush()
                    gt_V.append(v)
                    lp.add_gt(gt_offline[gt_idx]["t_ms"], v)
                    gt_idx += 1

            # keys
            try:
                k = keyq.get_nowait()
            except queue.Empty:
                k = None
            if k == "q":
                print("[EXIT] Bye.")
                break
            elif k == "o":
                path = input("GT CSV path: ").strip()
                if os.path.exists(path):
                    gt_offline[:] = load_gt_csv(path)
                    gt_idx = 0
                    print(f"[GT] Loaded {len(gt_offline)} rows from {path}")
                else:
                    print("[GT] File not found.")
            elif k == "g":
                run_fit()

            plt.pause(0.001)
    finally:
        try: f.close()
        except: pass
        try: ser_st and ser_st.close()
        except: pass
        try: ser_gt and ser_gt.close()
        except: pass

if __name__ == "__main__":
    main()
