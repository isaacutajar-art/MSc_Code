#!/usr/bin/env python3
"""VL53L7CX CNN distance-only human detection with 1-of-2 ON / 5-negative OFF.

Preserves per-cluster distance/velocity tracking and ZeroMQ JSON compatible
with ssm2.py. No heatmap. Research prototype; not safety-rated.


python3 publisher_zeromq_updated.py \
    model.pt \
    scaler_x.joblib \
    scaler_distance.joblib \
    normal_zone_reference.csv \
    --robot-ip 192.168.1.102 \
    --udp-port 8888 \
    --zmq-bind "tcp://*:5555" \
    --output runtime_detection_log.csv
"""
from __future__ import annotations
import argparse
import csv
import signal
import socket
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque
import joblib
import numpy as np
import pandas as pd
import torch
from torch import nn

PACKET_MAGIC = 0x4C375358
PACKET_VERSION = 1
SENSOR_COUNT = 6
ZONES = 64
GRID_H = GRID_W = 8
OUTPUT_SIZE = SENSOR_COUNT * ZONES
VALID_STATUS = 5
HEADER_STRUCT = struct.Struct("<I H I I B")
SENSOR_BLOCK_STRUCT = struct.Struct("<B B B B B H I 64H 64B")
HEADER_SIZE = HEADER_STRUCT.size
SENSOR_BLOCK_SIZE = SENSOR_BLOCK_STRUCT.size
EXPECTED_PACKET_SIZE = HEADER_SIZE + SENSOR_COUNT * SENSOR_BLOCK_SIZE
UDP_IP = "0.0.0.0"
UDP_PORT = 8888
PHYSICAL_SENSOR_RING = (1,4,3,6,5,2)


def input_cols() -> list[str]:
    return [
        "q0_rad", "q1_rad", "q2_rad", "q3_rad", "q4_rad", "q5_rad",
        "tcp_x_m", "tcp_y_m", "tcp_z_m", "tcp_rx", "tcp_ry", "tcp_rz",
    ]


@dataclass(frozen=True)
class SensorBlock:
    sensor_id: int
    fresh: int
    init_ok: int
    range_ok: int
    data_ok: int
    frame_seq: int
    timestamp_ms: int
    distance_mm: tuple[int, ...]
    status: tuple[int, ...]


@dataclass(frozen=True)
class Packet:
    packet_seq: int
    timestamp_ms: int
    sensors: tuple[SensorBlock, ...]


@dataclass(frozen=True)
class RobotState:
    sample_perf_s: float
    q: tuple[float, float, float, float, float, float]
    tcp: tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class ClusterInfo:
    sensor_id: int
    size: int
    distance_count: int
    status_count: int
    # Global flattened zone indices (0..383) belonging to this component.
    # Retained so the live heatmap can show exactly which pixels formed a cluster.
    zone_indices: tuple[int, ...]


@dataclass(frozen=True)
class FrameDetection:
    human_raw: bool
    state: str

    # Largest component in the mask used by the selected fusion mode,
    # regardless of whether it satisfies the final HUMAN rule.
    largest_cluster: int
    cluster_sensor: int | None
    largest_cluster_distance_count: int
    largest_cluster_status_count: int

    # Component that actually triggered HUMAN. Zero/None if no component qualifies.
    trigger_cluster: int
    trigger_sensor: int | None
    trigger_distance_count: int
    trigger_status_count: int

    distance_anomaly_zones: int
    status_anomaly_zones: int
    combined_anomaly_zones: int
    clear_count: int
    max_distance_score: float
    inference_ms: float
    processing_ms: float


def parse_packet(data: bytes) -> Packet | None:
    if len(data) != EXPECTED_PACKET_SIZE:
        return None

    magic, version, packet_seq, timestamp_ms, sensor_count = HEADER_STRUCT.unpack_from(data, 0)
    if magic != PACKET_MAGIC or version != PACKET_VERSION or sensor_count != SENSOR_COUNT:
        return None

    sensors: list[SensorBlock] = []
    offset = HEADER_SIZE

    for _ in range(sensor_count):
        unpacked = SENSOR_BLOCK_STRUCT.unpack_from(data, offset)
        offset += SENSOR_BLOCK_SIZE

        sensor_id, fresh, init_ok, range_ok, data_ok, frame_seq, sensor_ts = unpacked[:7]
        distances = tuple(int(v) for v in unpacked[7:7 + ZONES])
        statuses = tuple(int(v) for v in unpacked[7 + ZONES:7 + 2 * ZONES])

        sensors.append(
            SensorBlock(
                sensor_id=int(sensor_id),
                fresh=int(fresh),
                init_ok=int(init_ok),
                range_ok=int(range_ok),
                data_ok=int(data_ok),
                frame_seq=int(frame_seq),
                timestamp_ms=int(sensor_ts),
                distance_mm=distances,
                status=statuses,
            )
        )

    return Packet(
        packet_seq=int(packet_seq),
        timestamp_ms=int(timestamp_ms),
        sensors=tuple(sensors),
    )


def packet_to_arrays(pkt: Packet) -> tuple[np.ndarray, np.ndarray]:
    blocks = {b.sensor_id: b for b in pkt.sensors}

    distance = np.full(OUTPUT_SIZE, np.nan, dtype=np.float32)
    status = np.full(OUTPUT_SIZE, 255, dtype=np.int16)

    for sensor_id in range(1, SENSOR_COUNT + 1):
        block = blocks.get(sensor_id)
        if block is None:
            continue

        start = (sensor_id - 1) * ZONES
        end = start + ZONES
        distance[start:end] = np.asarray(block.distance_mm, dtype=np.float32)
        status[start:end] = np.asarray(block.status, dtype=np.int16)

    return distance, status


class RTDEStateBuffer:
    """Background UR RTDE reader with timestamped pose buffer."""

    def __init__(
        self,
        robot_ip: str,
        frequency_hz: float,
        stop_event: threading.Event,
        buffer_seconds: float = 2.0,
    ):
        self.robot_ip = robot_ip
        self.frequency_hz = float(frequency_hz)
        self.stop_event = stop_event
        maxlen = max(100, int(max(self.frequency_hz, 1.0) * buffer_seconds))
        self.states: Deque[RobotState] = deque(maxlen=maxlen)
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.connected = False

    def start(self) -> None:
        self.thread.start()

    def join(self, timeout: float | None = None) -> None:
        self.thread.join(timeout=timeout)

    def _run(self) -> None:
        try:
            import rtde_receive
        except ImportError:
            print("ERROR: install ur-rtde with: pip install ur-rtde", file=sys.stderr)
            self.stop_event.set()
            return

        rtde_r = None
        period_s = 1.0 / self.frequency_hz if self.frequency_hz > 0 else 0.002

        while not self.stop_event.is_set():
            try:
                if rtde_r is None:
                    print(f"Connecting to UR RTDE at {self.robot_ip}...")
                    rtde_r = rtde_receive.RTDEReceiveInterface(
                        self.robot_ip,
                        self.frequency_hz,
                    )
                    self.connected = True
                    print("UR RTDE connected.")

                t0 = time.perf_counter()
                q = tuple(float(v) for v in rtde_r.getActualQ())
                tcp = tuple(float(v) for v in rtde_r.getActualTCPPose())

                with self.lock:
                    self.states.append(RobotState(t0, q, tcp))

                elapsed = time.perf_counter() - t0
                sleep_s = period_s - elapsed
                if sleep_s > 0:
                    time.sleep(sleep_s)

            except Exception as exc:
                self.connected = False
                print(f"RTDE error: {exc}. Reconnecting...", file=sys.stderr)
                try:
                    if rtde_r is not None:
                        rtde_r.disconnect()
                except Exception:
                    pass
                rtde_r = None
                time.sleep(1.0)

        try:
            if rtde_r is not None:
                rtde_r.disconnect()
        except Exception:
            pass

    def nearest(self, target_perf_s: float) -> RobotState | None:
        with self.lock:
            if not self.states:
                return None
            states = tuple(self.states)

        return min(states, key=lambda s: abs(s.sample_perf_s - target_perf_s))



class PoseToMapsCNN(nn.Module):
    def __init__(self, in_channels=12, output_size=OUTPUT_SIZE, dropout=0.1):
        super().__init__()
        self.backbone = nn.Sequential(
            nn.Conv1d(in_channels,128,1),nn.BatchNorm1d(128),nn.GELU(),
            nn.Conv1d(128,256,1),nn.BatchNorm1d(256),nn.GELU(),
            nn.Conv1d(256,512,1),nn.BatchNorm1d(512),nn.GELU(),
            nn.Flatten(),nn.Linear(512,512),nn.GELU(),nn.Dropout(dropout),
            nn.Linear(512,512),nn.GELU(),nn.Dropout(dropout))
        self.distance_head = nn.Sequential(nn.Linear(512,512),nn.GELU(),nn.Linear(512,output_size))
        self.status_head = nn.Sequential(nn.Linear(512,512),nn.GELU(),nn.Linear(512,output_size))
    def forward(self,x):
        h=self.backbone(x.unsqueeze(-1))
        return self.distance_head(h),self.status_head(h)

class RuntimeModel:
    def __init__(self, model_file, scaler_x_file, scaler_distance_file, device):
        self.device=device
        try:
            checkpoint=torch.load(model_file,map_location=device,weights_only=False)
        except TypeError:
            checkpoint=torch.load(model_file,map_location=device)
        if not isinstance(checkpoint,dict) or 'model_state_dict' not in checkpoint:
            raise ValueError('Expected checkpoint containing model_state_dict')
        if int(checkpoint.get('output_size',OUTPUT_SIZE))!=OUTPUT_SIZE:
            raise ValueError('Unexpected model output size')
        if list(checkpoint.get('input_cols',input_cols()))!=input_cols():
            raise ValueError('Model input columns do not match the detector')
        self.scaler_x=joblib.load(scaler_x_file)
        self.scaler_distance=joblib.load(scaler_distance_file)
        self.model=PoseToMapsCNN(len(input_cols()),OUTPUT_SIZE,float(checkpoint.get('dropout',0.1))).to(device)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.model.eval()
        print(f"Loaded model {model_file} (training samples requested: {checkpoint.get('requested_dataset_size','unknown')})")
    def predict(self,state):
        x=np.asarray([list(state.q)+list(state.tcp)],dtype=np.float32)
        x=self.scaler_x.transform(x).astype(np.float32)
        t0=time.perf_counter()
        with torch.inference_mode():
            y,_=self.model(torch.from_numpy(x).to(self.device))
            if self.device.type=='cuda': torch.cuda.synchronize(self.device)
        inference_ms=1000*(time.perf_counter()-t0)
        dist=self.scaler_distance.inverse_transform(y.detach().cpu().numpy().reshape(-1,1)).reshape(-1)
        return dist.astype(np.float32),inference_ms

def load_reference_map(path):
    df=pd.read_csv(path)
    needed={'sensor_id','zone_id','P95_absolute_error_mm'}
    if not needed.issubset(df.columns):
        raise ValueError(f"Missing reference CSV columns: {needed-set(df.columns)}")
    p95=np.full(OUTPUT_SIZE,np.nan,dtype=np.float32)
    usable=np.zeros(OUTPUT_SIZE,dtype=bool)
    for row in df.itertuples(index=False):
        sid=int(getattr(row,'sensor_id')); zid=int(getattr(row,'zone_id'))
        if not(1<=sid<=SENSOR_COUNT and 0<=zid<ZONES): continue
        v=pd.to_numeric(getattr(row,'P95_absolute_error_mm'),errors='coerce')
        if not np.isfinite(v) or v<=0: continue
        flag=getattr(row,'reference_usable',True)
        if isinstance(flag,str): flag=flag.strip().lower() in ('true','1','yes')
        else: flag=bool(flag) and not pd.isna(flag)
        idx=(sid-1)*ZONES+zid
        p95[idx]=float(v); usable[idx]=flag
    print(f'Usable calibration zones: {int(usable.sum())}/{OUTPUT_SIZE}')
    if not usable.any(): raise ValueError('No usable calibration zones')
    return p95,usable

class TemporalHumanState:
    def __init__(self,on_window=2,on_required=1,off_required=5):
        if on_window<1 or not 1<=on_required<=on_window or off_required<1:
            raise ValueError('Invalid temporal frame configuration')
        self.recent=deque(maxlen=on_window)
        self.on_required=on_required
        self.off_required=off_required
        self.human_detected=False
        self.consecutive_negative=0
    def update(self,spatial_detected):
        self.recent.append(bool(spatial_detected))
        if not self.human_detected:
            if sum(self.recent)>=self.on_required:
                self.human_detected=True
                self.consecutive_negative=0
        elif spatial_detected:
            self.consecutive_negative=0
        else:
            self.consecutive_negative+=1
            if self.consecutive_negative>=self.off_required:
                self.human_detected=False
                self.consecutive_negative=0
                self.recent.clear()
        return self.human_detected

class HumanDetector:
    def __init__(self, p95, usable, threshold=3.0, cluster_size=3, on_window=2,
                 on_required=1, off_required=5):
        self.p95=p95; self.usable=usable
        self.threshold=threshold;self.cluster_size=cluster_size
        self.temporal=TemporalHumanState(on_window,on_required,off_required)
    def detect(self,actual_distance,actual_status,pred_distance,inference_ms,processing_start_s):
        valid=(actual_status==VALID_STATUS)&self.usable&np.isfinite(actual_distance)&np.isfinite(pred_distance)&np.isfinite(self.p95)&(self.p95>0)
        sd=np.full(OUTPUT_SIZE,np.nan,dtype=np.float32)
        sd[valid]=np.abs(actual_distance[valid]-pred_distance[valid])/self.p95[valid]
        anomalies=np.isfinite(sd)&(sd>self.threshold) # STRICT > as in new detector
        infos,largest_per_sensor=cluster_infos_per_sensor(anomalies,anomalies,np.zeros(OUTPUT_SIZE,dtype=bool),4)
        qualifying=[c for c in infos if c.size>=self.cluster_size]
        raw=bool(qualifying)
        human=self.temporal.update(raw)
        largest=infos[0] if infos else None
        trigger=qualifying[0] if qualifying else None
        max_sd=float(np.max(sd[np.isfinite(sd)])) if np.isfinite(sd).any() else float('nan')
        result=FrameDetection(
            human_raw=raw,state='HUMAN' if human else 'CLEAR',
            largest_cluster=largest.size if largest else 0,
            cluster_sensor=largest.sensor_id if largest else None,
            largest_cluster_distance_count=largest.size if largest else 0,
            largest_cluster_status_count=0,
            trigger_cluster=trigger.size if trigger else 0,
            trigger_sensor=trigger.sensor_id if trigger else None,
            trigger_distance_count=trigger.size if trigger else 0,
            trigger_status_count=0,
            distance_anomaly_zones=int(anomalies.sum()),status_anomaly_zones=0,
            combined_anomaly_zones=int(anomalies.sum()),
            clear_count=self.temporal.consecutive_negative,
            max_distance_score=max_sd,inference_ms=inference_ms,
            processing_ms=(time.perf_counter()-processing_start_s)*1000)
        return result,{'qualifying_clusters':qualifying,'largest_per_sensor':largest_per_sensor}


def connected_components(
    binary_map: np.ndarray,
    connectivity: int = 8,
) -> list[np.ndarray]:
    """
    Return connected components as arrays of [row, col] coordinates.
    """
    binary = np.asarray(binary_map, dtype=bool)

    if binary.shape != (GRID_H, GRID_W):
        raise ValueError(
            f"Expected {GRID_H}x{GRID_W}, got {binary.shape}."
        )

    if connectivity == 4:
        neighbours = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
        ]
    elif connectivity == 8:
        neighbours = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),            (0, 1),
            (1, -1),  (1, 0),   (1, 1),
        ]
    else:
        raise ValueError("connectivity must be 4 or 8")

    visited = np.zeros_like(binary, dtype=bool)
    components: list[np.ndarray] = []

    for row in range(GRID_H):
        for col in range(GRID_W):
            if not binary[row, col] or visited[row, col]:
                continue

            q = deque([(row, col)])
            visited[row, col] = True
            coords: list[tuple[int, int]] = []

            while q:
                r, c = q.popleft()
                coords.append((r, c))

                for dr, dc in neighbours:
                    nr = r + dr
                    nc = c + dc

                    if (
                        0 <= nr < GRID_H
                        and 0 <= nc < GRID_W
                        and binary[nr, nc]
                        and not visited[nr, nc]
                    ):
                        visited[nr, nc] = True
                        q.append((nr, nc))

            components.append(
                np.asarray(coords, dtype=np.int32)
            )

    return components


def cluster_infos_per_sensor(
    fusion_mask_flat: np.ndarray,
    distance_mask_flat: np.ndarray,
    status_mask_flat: np.ndarray,
    connectivity: int,
) -> tuple[list[ClusterInfo], list[int]]:
    """
    Cluster EACH 8x8 sensor independently.

    For every component, also count how many of its pixels came from:
      - distance evidence B_d
      - status evidence B_s

    This lets us test:
      OR:
        cluster(B_d OR B_s) >= C

      AND:
        cluster(B_d AND B_s) >= C

      OR+status:
        cluster(B_d OR B_s) >= C
        AND that same cluster contains >= N status-anomaly pixels
    """
    for name, arr in [
        ("fusion_mask_flat", fusion_mask_flat),
        ("distance_mask_flat", distance_mask_flat),
        ("status_mask_flat", status_mask_flat),
    ]:
        if np.asarray(arr).shape != (OUTPUT_SIZE,):
            raise ValueError(
                f"{name} must have shape ({OUTPUT_SIZE},), "
                f"got {np.asarray(arr).shape}."
            )

    infos: list[ClusterInfo] = []
    largest_per_sensor: list[int] = []

    for sensor_id in range(1, SENSOR_COUNT + 1):
        start = (sensor_id - 1) * ZONES
        end = start + ZONES

        fusion_map = np.asarray(
            fusion_mask_flat[start:end],
            dtype=bool,
        ).reshape(GRID_H, GRID_W)

        distance_map = np.asarray(
            distance_mask_flat[start:end],
            dtype=bool,
        ).reshape(GRID_H, GRID_W)

        status_map = np.asarray(
            status_mask_flat[start:end],
            dtype=bool,
        ).reshape(GRID_H, GRID_W)

        components = connected_components(
            fusion_map,
            connectivity=connectivity,
        )

        sensor_largest = 0

        for coords in components:
            rows = coords[:, 0]
            cols = coords[:, 1]

            size = int(len(coords))
            distance_count = int(
                np.sum(distance_map[rows, cols])
            )
            status_count = int(
                np.sum(status_map[rows, cols])
            )

            local_indices = rows * GRID_W + cols
            global_indices = tuple(
                int(start + idx)
                for idx in local_indices.tolist()
            )

            infos.append(
                ClusterInfo(
                    sensor_id=sensor_id,
                    size=size,
                    distance_count=distance_count,
                    status_count=status_count,
                    zone_indices=global_indices,
                )
            )

            sensor_largest = max(
                sensor_largest,
                size,
            )

        largest_per_sensor.append(sensor_largest)

    infos.sort(
        key=lambda c: (
            c.size,
            c.status_count,
            c.distance_count,
        ),
        reverse=True,
    )

    return infos, largest_per_sensor


def cluster_qualifies(
    cluster: ClusterInfo,
    fusion_mode: str,
    cluster_size: int,
    min_status_in_cluster: int,
) -> bool:
    if cluster.size < cluster_size:
        return False

    if fusion_mode == "or_status":
        return (
            cluster.status_count
            >= min_status_in_cluster
        )

    # For "or" and "and", the cluster-size rule alone is the
    # final component-level condition. The selected mask itself
    # defines whether the component is OR or AND evidence.
    return True



@dataclass(frozen=True)
class ClusterObservation:
    """One qualifying HUMAN cluster in one runtime frame."""

    sensor_id: int
    zone_indices: tuple[int, ...]
    centroid_row: float
    centroid_col: float
    min_distance_mm: float
    median_distance_mm: float


@dataclass
class HumanTrack:
    """Persistent ID plus range history for one detected HUMAN cluster."""

    track_id: int
    sensor_id: int
    centroid_row: float
    centroid_col: float
    min_distance_mm: float
    median_distance_mm: float
    last_seen_s: float
    zone_indices: tuple[int, ...]
    age_frames: int = 1
    missed_frames: int = 0
    velocity_raw_mps: float = float("nan")
    velocity_filtered_mps: float = float("nan")
    history: Deque[tuple[float, float]] = field(default_factory=deque)


class HumanClusterTracker:
    """
    Track qualifying HUMAN clusters and estimate relative radial/closing velocity.

    Sign convention:
        + velocity  -> separation is decreasing (CLOSING)
        - velocity  -> separation is increasing (OPENING)

    Velocity is estimated from a least-squares line fitted to the recent MEDIAN
    cluster distance history. The minimum cluster distance is retained separately
    for later SSM separation use.

    Important: if a track hands off to a neighbouring physical sensor, the same ID
    can be retained, but the velocity history is reset because the measurement
    origin changed from one ToF sensor to another. This avoids a false velocity
    spike caused by sensor-to-sensor mounting offsets.
    """

    def __init__(
        self,
        max_zone_jump: float = 3.0,
        max_distance_jump_mm: float = 250.0,
        max_missed_frames: int = 3,
        velocity_window: int = 8,
        velocity_alpha: float = 0.35,
        allow_adjacent_sensor_handoff: bool = True,
    ):
        self.max_zone_jump = float(max_zone_jump)
        self.max_distance_jump_mm = float(max_distance_jump_mm)
        self.max_missed_frames = int(max_missed_frames)
        self.velocity_window = max(3, int(velocity_window))
        self.velocity_alpha = float(velocity_alpha)
        self.allow_adjacent_sensor_handoff = bool(allow_adjacent_sensor_handoff)

        self.next_track_id = 1
        self.tracks: dict[int, HumanTrack] = {}

        self._adjacent_pairs: set[frozenset[int]] = set()
        ring = list(PHYSICAL_SENSOR_RING)
        for i, sensor_id in enumerate(ring):
            neighbour = ring[(i + 1) % len(ring)]
            self._adjacent_pairs.add(frozenset((sensor_id, neighbour)))

    def _are_adjacent(self, a: int, b: int) -> bool:
        if a == b:
            return True
        return frozenset((int(a), int(b))) in self._adjacent_pairs

    @staticmethod
    def _build_observation(
        cluster: ClusterInfo,
        actual_distance: np.ndarray,
    ) -> ClusterObservation | None:
        indices = np.asarray(cluster.zone_indices, dtype=np.int32)
        if indices.size == 0:
            return None

        values = np.asarray(actual_distance[indices], dtype=np.float64)
        finite = np.isfinite(values) & (values > 0.0)
        if not np.any(finite):
            return None

        values = values[finite]

        local = indices % ZONES
        rows = local // GRID_W
        cols = local % GRID_W

        return ClusterObservation(
            sensor_id=int(cluster.sensor_id),
            zone_indices=tuple(int(x) for x in indices.tolist()),
            centroid_row=float(np.mean(rows)),
            centroid_col=float(np.mean(cols)),
            min_distance_mm=float(np.min(values)),
            median_distance_mm=float(np.median(values)),
        )

    def _association_cost(
        self,
        track: HumanTrack,
        obs: ClusterObservation,
    ) -> float | None:
        distance_jump = abs(
            obs.median_distance_mm - track.median_distance_mm
        )
        if distance_jump > self.max_distance_jump_mm:
            return None

        if obs.sensor_id == track.sensor_id:
            zone_jump = float(
                np.hypot(
                    obs.centroid_row - track.centroid_row,
                    obs.centroid_col - track.centroid_col,
                )
            )
            if zone_jump > self.max_zone_jump:
                return None

            return (
                zone_jump / max(self.max_zone_jump, 1e-6)
                + distance_jump / max(self.max_distance_jump_mm, 1e-6)
            )

        if (
            self.allow_adjacent_sensor_handoff
            and self._are_adjacent(track.sensor_id, obs.sensor_id)
        ):
            # Pixel coordinates are not directly comparable across sensor frames,
            # so sensor handoff is matched using range continuity plus a penalty.
            return (
                1.25
                + distance_jump / max(self.max_distance_jump_mm, 1e-6)
            )

        return None

    @staticmethod
    def _fit_closing_velocity(history: Deque[tuple[float, float]]) -> float:
        if len(history) < 3:
            return float("nan")

        samples = list(history)
        t = np.asarray([x[0] for x in samples], dtype=np.float64)
        d_mm = np.asarray([x[1] for x in samples], dtype=np.float64)

        t = t - t[0]
        if t[-1] <= 0.02:
            return float("nan")

        # d(t) = slope*t + intercept. Closing velocity is -slope.
        slope_mm_s = float(np.polyfit(t, d_mm, 1)[0])
        return -slope_mm_s / 1000.0

    def _new_track(
        self,
        obs: ClusterObservation,
        timestamp_s: float,
    ) -> HumanTrack:
        track = HumanTrack(
            track_id=self.next_track_id,
            sensor_id=obs.sensor_id,
            centroid_row=obs.centroid_row,
            centroid_col=obs.centroid_col,
            min_distance_mm=obs.min_distance_mm,
            median_distance_mm=obs.median_distance_mm,
            last_seen_s=float(timestamp_s),
            zone_indices=obs.zone_indices,
        )
        track.history = deque(maxlen=self.velocity_window)
        track.history.append((float(timestamp_s), obs.median_distance_mm))
        self.tracks[track.track_id] = track
        self.next_track_id += 1
        return track

    def _update_track(
        self,
        track: HumanTrack,
        obs: ClusterObservation,
        timestamp_s: float,
    ) -> None:
        sensor_changed = obs.sensor_id != track.sensor_id

        # If time moves backwards (e.g. Teensy reset/wrap) start a new velocity
        # history rather than generating a bad derivative.
        time_invalid = float(timestamp_s) <= float(track.last_seen_s)

        track.sensor_id = obs.sensor_id
        track.centroid_row = obs.centroid_row
        track.centroid_col = obs.centroid_col
        track.min_distance_mm = obs.min_distance_mm
        track.median_distance_mm = obs.median_distance_mm
        track.last_seen_s = float(timestamp_s)
        track.zone_indices = obs.zone_indices
        track.age_frames += 1
        track.missed_frames = 0

        if sensor_changed or time_invalid:
            track.history = deque(maxlen=self.velocity_window)
            track.history.append((float(timestamp_s), obs.median_distance_mm))
            track.velocity_raw_mps = float("nan")
            track.velocity_filtered_mps = float("nan")
            return

        track.history.append((float(timestamp_s), obs.median_distance_mm))
        raw = self._fit_closing_velocity(track.history)
        track.velocity_raw_mps = raw

        if np.isfinite(raw):
            if np.isfinite(track.velocity_filtered_mps):
                alpha = self.velocity_alpha
                track.velocity_filtered_mps = (
                    alpha * raw
                    + (1.0 - alpha) * track.velocity_filtered_mps
                )
            else:
                track.velocity_filtered_mps = raw

    def update(
        self,
        qualifying_clusters: list[ClusterInfo],
        actual_distance: np.ndarray,
        timestamp_s: float,
    ) -> tuple[list[HumanTrack], list[HumanTrack], np.ndarray]:
        observations = [
            obs
            for cluster in qualifying_clusters
            if (
                obs := self._build_observation(
                    cluster,
                    actual_distance,
                )
            ) is not None
        ]

        # Mark all existing tracks unseen until associated this frame.
        for track in self.tracks.values():
            track.missed_frames += 1

        candidates: list[tuple[float, int, int]] = []
        for track_id, track in self.tracks.items():
            if track.missed_frames > self.max_missed_frames + 1:
                continue

            for obs_idx, obs in enumerate(observations):
                cost = self._association_cost(track, obs)
                if cost is not None:
                    candidates.append((cost, track_id, obs_idx))

        candidates.sort(key=lambda x: x[0])
        used_tracks: set[int] = set()
        used_obs: set[int] = set()
        obs_to_track: dict[int, HumanTrack] = {}

        for _cost, track_id, obs_idx in candidates:
            if track_id in used_tracks or obs_idx in used_obs:
                continue

            track = self.tracks[track_id]
            obs = observations[obs_idx]
            self._update_track(track, obs, timestamp_s)

            used_tracks.add(track_id)
            used_obs.add(obs_idx)
            obs_to_track[obs_idx] = track

        for obs_idx, obs in enumerate(observations):
            if obs_idx in used_obs:
                continue
            track = self._new_track(obs, timestamp_s)
            obs_to_track[obs_idx] = track

        # Remove tracks that have not been seen for too long.
        expired = [
            track_id
            for track_id, track in self.tracks.items()
            if track.missed_frames > self.max_missed_frames
        ]
        for track_id in expired:
            del self.tracks[track_id]

        current_tracks = [
            obs_to_track[i]
            for i in range(len(observations))
            if i in obs_to_track
        ]
        current_tracks.sort(key=lambda t: t.track_id)

        active_tracks = sorted(
            self.tracks.values(),
            key=lambda t: t.track_id,
        )

        track_zone_ids = np.zeros(OUTPUT_SIZE, dtype=np.int32)
        for track in current_tracks:
            if track.zone_indices:
                track_zone_ids[
                    np.asarray(track.zone_indices, dtype=np.int32)
                ] = int(track.track_id)

        return current_tracks, active_tracks, track_zone_ids


