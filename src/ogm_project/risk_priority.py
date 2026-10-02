"""Risk Score gates for offline priority analysis. No transmission operations."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .grid_contract import GridContract, GridMismatchError, read_metadata, validate_same_grid
from .logodds import OCC_TH, FREE_TH, probability_from_logodds as _probability


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def probability_from_logodds(logodds):
    a = np.asarray(logodds)
    if a.ndim != 2 or a.dtype.kind != 'f' or not np.isfinite(a).all():
        raise ValueError('logodds must be a finite 2D floating array')
    # Same operation/dtype as the existing implementation; extreme finite
    # inputs may overflow exp, whose sigmoid result is safely zero.
    with np.errstate(over='ignore'):
        return _probability(a)


def compute_known_masks(ego_logodds, rsu_logodds):
    p_ego, p_rsu = probability_from_logodds(ego_logodds), probability_from_logodds(rsu_logodds)
    if p_ego.shape != p_rsu.shape:
        raise ValueError('Ego/RSU shape mismatch')
    ego_known = (p_ego >= OCC_TH) | (p_ego <= FREE_TH)
    rsu_known = (p_rsu >= OCC_TH) | (p_rsu <= FREE_TH)
    return ~ego_known, rsu_known


def boolean_mask(value, shape, name):
    a = np.asarray(value)
    if a.dtype != np.bool_ or a.shape != shape:
        raise ValueError(f'{name}: expected bool {shape}, got {a.dtype} {a.shape}')
    return a


def baseline_candidate(ego_unknown, rsu_known, road_mask):
    a = np.asarray(ego_unknown)
    if a.ndim != 2:
        raise ValueError('masks must be 2D')
    return (boolean_mask(a, a.shape, 'ego_unknown') &
            boolean_mask(rsu_known, a.shape, 'rsu_known') & boolean_mask(road_mask, a.shape, 'road_mask'))


def compute_priority_map(risk, ego_unknown, rsu_known, road_mask):
    risk = np.asarray(risk)
    if risk.ndim != 2 or risk.dtype != np.float32 or not np.isfinite(risk).all():
        raise ValueError('Risk Score must be a finite 2D float32 array')
    if np.any((risk < 0) | (risk > 1)):
        raise ValueError('Risk Score must be in [0,1]')
    for name, mask in [('ego_unknown', ego_unknown), ('rsu_known', rsu_known), ('road_mask', road_mask)]:
        boolean_mask(mask, risk.shape, name)
    return (risk.astype(np.float32) * ego_unknown.astype(np.float32) *
            rsu_known.astype(np.float32) * road_mask.astype(np.float32))


def load_road_mask(array_path, metadata_path, grid):
    meta = read_metadata(metadata_path)
    validate_same_grid(grid, GridContract.from_metadata(meta))
    if meta.get('array_sha256') != sha256(array_path):
        raise GridMismatchError('mask array_sha256 missing or does not match the NPY file')
    mask = boolean_mask(np.load(array_path, allow_pickle=False), grid.shape, 'mask')
    semantics = meta.get('mask_semantics')
    if semantics == 'road_true':
        road = mask
    elif semantics == 'static_non_driving_true':
        road = ~mask
    else:
        raise GridMismatchError('mask_semantics must be road_true or static_non_driving_true')
    if not isinstance(meta.get('provenance'), str) or not meta['provenance'].strip():
        raise GridMismatchError('mask provenance is required; do not infer it from shape')
    return road, meta


def experiment_alignment(risk_meta, ogm_meta):
    modern_keys = ('experiment_id', 'scenario_name', 'map_name', 'ego_initial_state',
        'target_initial_state', 'rsu_pose', 'grid_center_x', 'grid_center_y',
        'resolution_m', 'nx', 'ny', 'fixed_delta_seconds', 'prediction_horizon_s',
        'random_seed', 'tm_seed', 'reference_frame', 'reference_simulation_time_s',
        'time_reference_id')
    keys = ('scenario_id', 'scenario_sha256', 'ego_initial_pose', 'target_id', 'time_reference_id', 'reference_time_s')
    a, b = risk_meta.get('experiment') or {}, ogm_meta.get('experiment') or {}
    if not isinstance(a, dict) or not isinstance(b, dict):
        raise ValueError('experiment metadata must be an object')
    def present(key, value):
        if key == 'ego_initial_pose':
            return isinstance(value, dict) and all(
                isinstance(value.get(k), (int, float)) and not isinstance(value[k], bool) and math.isfinite(value[k])
                for k in ('x', 'y', 'z', 'yaw'))
        if key == 'reference_time_s':
            return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0
        if key == 'scenario_sha256':
            return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)
        return isinstance(value, str) and bool(value.strip())
    if 'experiment_id' in a or 'experiment_id' in b:
        def valid_modern(key, value):
            if key in ('ego_initial_state', 'target_initial_state'):
                return isinstance(value, dict) and all(isinstance(value.get(k), (int, float)) and
                    not isinstance(value[k], bool) and math.isfinite(value[k]) for k in ('x','y','z','yaw','vx','vy','vz'))
            if key == 'rsu_pose':
                return isinstance(value, dict) and all(isinstance(value.get(k), (int, float)) and
                    not isinstance(value[k], bool) and math.isfinite(value[k]) for k in ('x','y','z','yaw'))
            if key in ('map_name','experiment_id','scenario_name','time_reference_id'):
                return isinstance(value, str) and bool(value.strip())
            if key in ('nx','ny','random_seed','tm_seed','reference_frame'):
                return type(value) is int and value >= 0
            return isinstance(value, (int,float)) and not isinstance(value,bool) and math.isfinite(value) and value >= 0
        missing = [k for k in modern_keys if not valid_modern(k, a.get(k)) or not valid_modern(k, b.get(k))]
        mismatches = [k for k in modern_keys if k not in missing and a[k] != b[k]]
        keys = modern_keys
    else:
        missing = [k for k in keys if not present(k, a.get(k)) or not present(k, b.get(k))]
        mismatches = [k for k in keys if k not in missing and a[k] != b[k]]
    return dict(compatible=False if mismatches else (None if missing else True),
                missing_fields=missing, mismatched_fields=mismatches,
                compared_fields=list(keys),
                status='mismatch' if mismatches else ('unverified' if missing else 'matched'))


def priority_summary(risk, ego_unknown, rsu_known, road, priority):
    baseline = baseline_candidate(ego_unknown, rsu_known, road)
    rp, pp = risk > 0, priority > 0
    nb, np_ = int(baseline.sum()), int(pp.sum())
    return dict(total_cells=int(risk.size), risk_positive_cells=int(rp.sum()),
        ego_unknown_cells=int(ego_unknown.sum()), rsu_known_cells=int(rsu_known.sum()), road_cells=int(road.sum()),
        candidate_cells_before_risk=nb, priority_positive_cells=np_, risk_max=float(risk.max()),
        priority_max=float(priority.max()), priority_mean_positive=float(priority[pp].mean()) if np_ else 0.,
        grid_compatible=True, baseline_candidate_cells=nb, risk_priority_cells=np_,
        reduction_ratio=1-np_/nb if nb else None, candidate_reduction_ratio=1-np_/nb if nb else None,
        reduction_definition='candidate cell count reduction; not transmitted bytes',
        risk_positive_and_ego_unknown=int((rp & ego_unknown).sum()),
        risk_positive_and_rsu_known=int((rp & rsu_known).sum()), risk_positive_and_road=int((rp & road).sum()),
        risk_and_all_conditions=np_)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def render_priority(out, grid, risk, ego_unknown, rsu_known, road, priority, baseline, ego_probability, rsu_probability):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    layers = [(ego_probability, 'Ego OGM probability'), (rsu_probability, 'RSU OGM probability'),
              (risk, 'Risk Score'), (ego_unknown, 'Ego Unknown'), (rsu_known, 'RSU Known'),
              (road, 'Road'), (baseline, 'Baseline candidate'), (priority, 'Priority Score')]
    fig, axes = plt.subplots(2, 4, figsize=(18, 9), layout='constrained')
    for ax, (data, title) in zip(axes.flat, layers):
        im = ax.imshow(data, origin='lower', extent=grid.extent, vmin=0, vmax=1, cmap='magma', interpolation='nearest')
        ax.set(title=title, xlabel='CARLA WORLD X [m]', ylabel='CARLA WORLD Y [m]', aspect='equal')
    fig.colorbar(im, ax=list(axes.flat), shrink=.8, label='Score / mask value')
    fig.savefig(out/'priority_components.png', dpi=150)
    fig.savefig(out/'priority_components_real_carla.png', dpi=150)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 7), layout='constrained')
    im = ax.imshow(priority, origin='lower', extent=grid.extent, vmin=0, vmax=1, cmap='magma', interpolation='nearest')
    ax.set(title='Communication Priority Map', xlabel='CARLA WORLD X [m]', ylabel='CARLA WORLD Y [m]', aspect='equal')
    fig.colorbar(im, ax=ax, label='Priority Score')
    fig.savefig(out/'priority_map.png', dpi=150)
    plt.close(fig)


def build_priority(ogm_dir, risk_dir, output_dir, *, require_experiment_match=False):
    ogm_dir, risk_dir, out = map(Path, (ogm_dir, risk_dir, output_dir))
    risk_meta = read_metadata(risk_dir/'metadata.json')
    ogm_meta = read_metadata(ogm_dir/'ogm_grid_metadata.json')
    risk_grid = GridContract.from_metadata(risk_meta, risk_v1=True)
    ogm_grid = GridContract.from_metadata(ogm_meta)
    validate_same_grid(risk_grid, ogm_grid)
    if ogm_meta.get('known_thresholds') != dict(occupied=OCC_TH, free=FREE_TH):
        raise ValueError('snapshot Known thresholds must match current Cooperative OGM')
    alignment = experiment_alignment(risk_meta, ogm_meta)
    if alignment['compatible'] is False or (require_experiment_match and alignment['compatible'] is not True):
        raise ValueError(f'ExperimentMismatchError: {alignment}')
    ego = np.load(ogm_dir/'ego_logodds.npy', allow_pickle=False)
    rsu = np.load(ogm_dir/'rsu_logodds.npy', allow_pickle=False)
    if ego.shape != ogm_grid.shape or rsu.shape != ogm_grid.shape:
        raise GridMismatchError('logodds array shapes do not match metadata')
    unknown, known = compute_known_masks(ego, rsu)
    # Check saved derived masks too; stale masks are not silently accepted.
    for name, expected in [('ego_unknown', unknown), ('rsu_known', known)]:
        saved = boolean_mask(np.load(ogm_dir/(name+'.npy'), allow_pickle=False), ogm_grid.shape, name)
        if not np.array_equal(saved, expected):
            raise ValueError(f'{name}: saved mask disagrees with logodds/current thresholds')
    road, mask_meta = load_road_mask(ogm_dir/'road_mask.npy', ogm_dir/'mask_metadata.json', ogm_grid)
    risk = np.load(risk_dir/'risk_aggregate.npy', allow_pickle=False)
    if risk.shape != risk_grid.shape:
        raise GridMismatchError('Risk array shape does not match metadata')
    priority = compute_priority_map(risk, unknown, known, road)
    baseline = baseline_candidate(unknown, known, road)
    summary = priority_summary(risk, unknown, known, road, priority)
    summary.update(experiment_compatible=alignment['compatible'], experiment_alignment=alignment,
                   research_interpretation='scenario_and_grid_matched' if alignment['compatible'] else 'grid_only_experiment_unverified')
    out.mkdir(parents=True, exist_ok=True)
    np.save(out/'priority_map.npy', priority)
    np.save(out/'baseline_candidate_mask.npy', baseline)
    write_json(out/'priority_summary.json', summary)
    write_json(out/'priority_metadata.json', dict(ogm_grid.metadata(),
        definition='risk_score * ego_unknown * rsu_known * road', known_thresholds=dict(occupied=OCC_TH, free=FREE_TH),
        source_risk_dir=str(risk_dir.resolve()), source_ogm_dir=str(ogm_dir.resolve()),
        source_mask_metadata=mask_meta, experiment_alignment=alignment,
        risk_sha256=sha256(risk_dir/'risk_aggregate.npy'),
        selective_transmission='NOT IMPLEMENTED'))
    render_priority(out, ogm_grid, risk, unknown, known, road, priority, baseline,
                    probability_from_logodds(ego), probability_from_logodds(rsu))
    return summary
