"""
Live 3D reconstruction of the actuator, driven directly by the Bend Labs ADS
sensor over BLE.
"""

import asyncio
import os
import struct
import threading
import time

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from bleak import BleakClient
from matplotlib.animation import FuncAnimation

# =================================================
# 1. BLE configuration
# =================================================

# This is to be changed for the specific sensor used.
if os.name == "nt":  # Windows
    TARGET_ADDRESS = "FD:64:D6:BE:BF:02"
elif os.name == "posix":  # macOS/Linux
    TARGET_ADDRESS = "CB544867-4D72-6F13-2E4B-2D33CA720E69"
else:
    raise OSError(f"Unrecognized OS {os.name}, nor 'nt' nor 'posix'.")

# BLE UUIDs for the ADS service and characteristic
ADS_SERVICE_UUID = "00001820-0000-1000-8000-00805f9b34fb"
ADS_ANGLE_CHAR_UUID = "00002a70-0000-1000-8000-00805f9b34fb"

PLOT_FPS = 30

# =================================================
# 2. Robot geometry
# =================================================

BASE_HEIGHT_MM = 14.8
SEGMENT_HEIGHTS_MM = [22, 20.8, 19.7, 18.5, 14.2]
VERTICAL_SPACING_MM = 1.0
BASE_DIAMETER = 37.0
TIP_DIAMETER = 14.0

segment_lengths = [h + VERTICAL_SPACING_MM for h in SEGMENT_HEIGHTS_MM]
L = sum(segment_lengths)
diameters = np.linspace(BASE_DIAMETER, TIP_DIAMETER, len(segment_lengths) + 1)[1:]


def reconstruct_structure(theta_x, theta_y):
    theta = np.sqrt(theta_x**2 + theta_y**2)
    phi = np.arctan2(theta_y, theta_x) if abs(theta) > 1e-6 else 0
    kappa = theta / L if theta >= 1e-6 else 0.0

    backbone_points = []
    s_val = 0.0
    for Li in segment_lengths:
        s_val += Li
        if kappa == 0:
            p = np.array([0.0, 0.0, s_val])
        else:
            R = 1.0 / kappa
            x = R * (1 - np.cos(kappa * s_val))
            z = R * np.sin(kappa * s_val)
            p = np.array([x * np.cos(phi), x * np.sin(phi), z])
        p[2] += BASE_HEIGHT_MM
        backbone_points.append(p)
    return backbone_points


def draw_cylinder(ax, p0, p1, radius, resolution=12):
    v = p1 - p0
    length = np.linalg.norm(v)
    if length == 0:
        return
    v = v / length
    not_v = np.array([1, 0, 0]) if abs(v[0]) < 0.9 else np.array([0, 1, 0])
    n1 = np.cross(v, not_v)
    n1 /= np.linalg.norm(n1)
    n2 = np.cross(v, n1)

    theta = np.linspace(0, 2 * np.pi, resolution)
    z = np.linspace(0, length, 2)
    theta, z = np.meshgrid(theta, z)

    X = (
        p0[0]
        + v[0] * z
        + radius * np.cos(theta) * n1[0]
        + radius * np.sin(theta) * n2[0]
    )
    Y = (
        p0[1]
        + v[1] * z
        + radius * np.cos(theta) * n1[1]
        + radius * np.sin(theta) * n2[1]
    )
    Z = (
        p0[2]
        + v[2] * z
        + radius * np.cos(theta) * n1[2]
        + radius * np.sin(theta) * n2[2]
    )

    ax.plot_surface(X, Y, Z, alpha=0.85, linewidth=0, color="tab:blue")


# =================================================
# 3. Shared state between the BLE thread and the plot thread
# =================================================

latest_angles = {"x": 0.0, "y": 0.0}
lock = threading.Lock()
stop_event = threading.Event()


def process_data_packet(data: bytearray) -> dict:
    data_length = len(data)
    current_time_ms = time.time() * 1000

    result = {
        "timestamp_ms": current_time_ms,
        "angle_x": None,
        "angle_y": None,
        "data_type": f"Unknown_{data_length}B",
    }

    try:
        if data_length == 2:
            angle_raw = struct.unpack("<h", data)[0]
            result["angle_x"] = angle_raw / 100.0
            result["data_type"] = "1-Axis (16-bit scaled)"
        elif data_length == 4:
            result["angle_x"] = struct.unpack("<f", data)[0]
            result["data_type"] = "1-Axis (32-bit float)"
        elif data_length == 8:
            angle_x, angle_y = struct.unpack("<ff", data)
            result["angle_x"] = angle_x
            result["angle_y"] = angle_y
            result["data_type"] = "2-Axis (32-bit floats)"
        return result
    except struct.error:
        result["data_type"] = f"Unpack_Error_{data_length}B"
        return result


# =================================================
# 4. BLE client
# =================================================

data_queue = None


def notification_handler(sender, data):
    try:
        data_queue.put_nowait(data)
    except asyncio.QueueFull:
        pass


async def data_consumer_task():

    baseline_x = None
    baseline_y = None

    while True:
        raw_data = await data_queue.get()
        processed = process_data_packet(raw_data)

        if processed["angle_x"] is not None:
            processed["raw_angle_x"] = processed["angle_x"]
            processed["raw_angle_y"] = processed["angle_y"]

            if baseline_x is None:
                baseline_x = processed["angle_x"]
                baseline_y = (
                    processed["angle_y"] if processed["angle_y"] is not None else 0.0
                )
                print(
                    f"\n*** BASELINE CALIBRATED: X={baseline_x:.2f}°, Y={baseline_y:.2f}° ***\n"
                )

            processed["angle_x"] -= baseline_x
            if processed["angle_y"] is not None:
                processed["angle_y"] -= baseline_y

        if processed["angle_x"] is not None:
            with lock:
                latest_angles["x"] = processed["angle_x"]
                if processed["angle_y"] is not None:
                    latest_angles["y"] = processed["angle_y"]

        data_queue.task_done()


async def run_ble_client():
    global data_queue
    data_queue = asyncio.Queue()
    consumer = asyncio.create_task(data_consumer_task())

    print(f"Connecting to {TARGET_ADDRESS}...")
    try:
        async with BleakClient(TARGET_ADDRESS) as client:
            if not client.is_connected:
                print("Failed to connect.")
                return

            print(f"Connected. Subscribing to {ADS_ANGLE_CHAR_UUID}...")
            await client.start_notify(ADS_ANGLE_CHAR_UUID, notification_handler)
            print("Streaming — close the plot window to stop.")

            while client.is_connected and not stop_event.is_set():
                await asyncio.sleep(0.1)

            await client.stop_notify(ADS_ANGLE_CHAR_UUID)
    except Exception as e:  # noqa: BLE001
        print(f"BLE error: {e}")
    finally:
        consumer.cancel()


def ble_thread_entry():
    try:
        asyncio.run(run_ble_client())
    except asyncio.CancelledError:
        pass


# =================================================
# 5. Live Presentation Plot
# =================================================

# Increased figure size for presentation visibility
fig = plt.figure(figsize=(11, 8))
fig.subplots_adjust(top=0.70)  # Leave room for the custom header

# Injecting Presentation Context
ax_header = fig.add_axes([0.25, 0.75, 0.50, 0.25])
ax_header.axis("off")  # Hides the border, background, and ticks
# 1. Title
ax_header.text(
    0.5,
    0.8,
    "Real-Time 3D Proprioception for Soft Robots\nUsing a Single Capacitive Bend Sensor",
    ha="center",
    va="center",
    fontsize=14,
    fontweight="bold",
    wrap=True,
    transform=ax_header.transAxes,
)

# 2. Authors
ax_header.text(
    0.5,
    0.45,
    "Marin R. Bricq, Emanuele Bianchi, Francesco Braghin,\nEmilia Ambrosini, and Marta Gandolla",
    ha="center",
    va="center",
    fontsize=11,
    wrap=True,
    transform=ax_header.transAxes,
)

# 3. Contact / Info
ax_header.text(
    0.5,
    0.15,
    "Contact: marinraymond.bricq@polimi.it  |  BioRob 2026 Live Demo",
    ha="center",
    va="center",
    fontsize=10,
    fontstyle="italic",
    color="dimgray",
    wrap=True,
    transform=ax_header.transAxes,
)

# Add PoliMi Logo (Left)
try:
    img_polimi = mpimg.imread("resources/polimi_logo.png")
    ax_logo_left = fig.add_axes([0.05, 0.86, 0.2, 0.1])
    ax_logo_left.imshow(img_polimi)
    ax_logo_left.axis("off")
except Exception as e:  # noqa: BLE001
    print(f"Warning: Could not load PoliMi logo - {e}")

# Add BioRob Logo (Right)
try:
    img_biorob = mpimg.imread("resources/biorob_logo.png")
    ax_logo_right = fig.add_axes([0.75, 0.86, 0.2, 0.1])
    ax_logo_right.imshow(img_biorob)
    ax_logo_right.axis("off")
except Exception as e:  # noqa: BLE001
    print(f"Warning: Could not load BioRob logo - {e}")

# The 3D plot underneath the header
ax = fig.add_subplot(111, projection="3d")


def update(frame):
    with lock:
        theta_x = np.deg2rad(latest_angles["x"])
        theta_y = np.deg2rad(latest_angles["y"])
        x_disp, y_disp = latest_angles["x"], latest_angles["y"]

    backbone = reconstruct_structure(theta_x, theta_y)

    ax.clear()
    p_prev = np.array([0.0, 0.0, BASE_HEIGHT_MM])
    for idx, p_curr in enumerate(backbone):
        radius = diameters[idx] / 2.0
        draw_cylinder(ax, p_prev, p_curr, radius)
        p_prev = p_curr

    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")

    # Subtitle for the live data readouts specifically
    ax.set_title(f"Live Sensor State | X: {x_disp:.1f}°  Y: {y_disp:.1f}°", pad=10)

    max_range = 120
    # Invert x axis to match the physical orientation of the robot
    ax.set_xlim(max_range, -max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(0, max_range * 1.5)
    ax.view_init(elev=25, azim=45)
    return (ax,)


if __name__ == "__main__":
    ble_thread = threading.Thread(target=ble_thread_entry, daemon=True)
    ble_thread.start()

    try:
        ani = FuncAnimation(
            fig, update, interval=1000 / PLOT_FPS, cache_frame_data=False
        )
        plt.show()
    finally:
        stop_event.set()
        ble_thread.join(timeout=3)
