"""Opt-in capture of one coherent Ego/local-RSU snapshot, without payload changes."""
from __future__ import annotations

from pathlib import Path
import math
import threading

import numpy as np

from .grid_contract import GridContract, read_metadata
from .risk_priority import compute_known_masks, load_road_mask, sha256, write_json
from .logodds import OCC_TH, FREE_TH


def preflight_mask(array_path, metadata_path):
    meta = read_metadata(metadata_path)
    load_road_mask(array_path, metadata_path, GridContract.from_metadata(meta))


class PrioritySnapshot:
    def __init__(self, out, grid_metadata, mask_path, mask_metadata_path, experiment):
        self.out = Path(out)
        self.metadata = dict(grid_metadata)
        self.grid = GridContract.from_metadata(self.metadata)
        self.road, self.mask_source = load_road_mask(mask_path, mask_metadata_path, self.grid)
        self.experiment = dict(experiment)
        self.lock = threading.Lock()
        self.measurements = {}
        self.saved = False

    def callback(self, name, callback, measurement):
        # Only debug mode takes this lock. Callbacks update the same arrays as
        # before; capture copies them only between completed sensor updates.
        with self.lock:
            self.measurements.pop(name, None)
            callback(measurement)
            if int(measurement.frame) < 0 or not math.isfinite(measurement.timestamp) or measurement.timestamp < 0:
                raise ValueError('invalid sensor frame/timestamp')
            self.measurements[name] = dict(frame=int(measurement.frame), timestamp=float(measurement.timestamp))

    def save_if_ready(self, ego, rsu):
        if self.saved:
            return False
        with self.lock:
            if set(self.measurements) != {'ego', 'rsu'}:
                return False
            a, b = self.measurements['ego'], self.measurements['rsu']
            if a != b:
                return False  # Do not label two different sensor frames as one instant.
            ego_copy, rsu_copy = ego.copy(), rsu.copy()
            measurements = {k: dict(v) for k, v in self.measurements.items()}
        if ego_copy.shape != self.grid.shape or rsu_copy.shape != self.grid.shape:
            raise ValueError('snapshot logodds/Grid shape mismatch')
        unknown, known = compute_known_masks(ego_copy, rsu_copy)
        self.out.mkdir(parents=True, exist_ok=True)
        for name, array in [('ego_logodds', ego_copy), ('rsu_logodds', rsu_copy),
                            ('ego_unknown', unknown), ('rsu_known', known), ('road_mask', self.road)]:
            np.save(self.out/(name+'.npy'), array)
        experiment = dict(self.experiment)
        experiment.setdefault('reference_frame', measurements['ego']['frame'])
        experiment.setdefault('reference_simulation_time_s', measurements['ego']['timestamp'])
        meta = dict(self.metadata, known_thresholds=dict(occupied=OCC_TH, free=FREE_TH),
            rsu_layer='local_before_quantization_and_transmission', sensor_measurements=measurements,
            snapshot_frame=measurements['ego']['frame'], snapshot_time_s=measurements['ego']['timestamp'],
            experiment=experiment)
        write_json(self.out/'experiment_metadata.json', experiment)
        write_json(self.out/'ogm_grid_metadata.json', meta)
        write_json(self.out/'mask_metadata.json', dict(self.grid.metadata(), mask_semantics='road_true',
            array_sha256=sha256(self.out/'road_mask.npy'), provenance='derived from validated source mask',
            source_mask_metadata=self.mask_source))
        self.saved = True
        return True
