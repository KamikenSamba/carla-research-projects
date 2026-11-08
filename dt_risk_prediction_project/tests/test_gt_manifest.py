"""Independent corruption tests, no real experiment or CARLA dependency."""
from pathlib import Path
import csv,hashlib,json,tempfile,unittest,builtins,os,subprocess,sys
from unittest.mock import patch
from gt_manifest_fixture import build
from dt_gt_manifest import validate

class ContractTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)/'bundle';self.path=build(self.root)
        self.m=json.loads(self.path.read_text(encoding='utf-8'))
    def tearDown(self):self.temp.cleanup()
    def check(self,expected=None):
        self.path.write_text(json.dumps(self.m),encoding='utf-8');r=validate(self.path)
        if expected:self.assertEqual(r['status'],'INVALID');self.assertTrue(any(expected in e for e in r['errors']),r)
        else:self.assertEqual(r['status'],'CONTRACT_VALID_PENDING_APPROVAL',r)
        self.assertFalse(r['independent_forecast_accuracy_enabled']);self.assertFalse(r['runtime_execution_enabled'])
        return r
    def edit_json(self,ref,change):
        p=self.root/ref['path'];d=json.loads(p.read_text());change(d);p.write_text(json.dumps(d),encoding='utf-8');ref['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
    def edit_csv(self,ref,change):
        p=self.root/ref['path']
        with p.open(newline='') as f:r=list(csv.DictReader(f))
        change(r)
        with p.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(r[0]));w.writeheader();w.writerows(r)
        ref['sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
    def test_valid_exact_two_actor_bundle(self):
        r=self.check();self.assertEqual(r['trajectory_samples_per_side'],200);self.assertTrue(r['causal_prefix_verified'])
    def test_no_carla_import(self):
        real=builtins.__import__
        def guard(name,*a,**kw):
            if name=='carla':raise AssertionError('CARLA import forbidden')
            return real(name,*a,**kw)
        with patch('builtins.__import__',guard):self.check()
    def test_wrong_parent(self):self.m['prediction']['parent_source_id']='other';self.check('PARENT_SOURCE')
    def test_wrong_snapshot_hash(self):self.m['prediction']['parent_snapshot_sha256']='f'*64;self.check('PARENT_SNAPSHOT_HASH')
    def test_artifact_tamper(self):
        (self.root/'source/snapshot.json').write_text('{}');self.check('HASH_MISMATCH')
    def test_reference_t0_mismatch(self):self.m['prediction']['reference_origin']['frame']=101;self.check('REFERENCE_T0')
    def test_local_t0_mismatch(self):self.m['prediction']['origin']['simulation_time_s']=100.1;self.check('SNAPSHOT_T0')
    def test_position_mismatch(self):
        self.edit_json(self.m['prediction']['artifacts']['snapshot'],lambda d:d['actors'][0].update(x=20.001))
        r=self.check();self.assertAlmostEqual(r['initial_residuals']['leader']['position_xyz_m'],.001)
        self.assertEqual(r['initial_state_assessment'],'NOT_EVALUATED_PENDING_POLICY')
    def test_velocity_mismatch(self):
        self.edit_json(self.m['prediction']['artifacts']['snapshot'],lambda d:d['actors'][0].update(vx=.6,speed_mps=.6))
        self.assertAlmostEqual(self.check()['initial_residuals']['leader']['velocity_xyz_mps'],.1)
    def test_heading_mismatch(self):
        self.edit_json(self.m['prediction']['artifacts']['snapshot'],lambda d:d['actors'][0].update(yaw=.01))
        self.assertAlmostEqual(self.check()['initial_residuals']['leader']['heading_deg'],.01)
    def test_logical_id_mismatch(self):self.m['prediction']['actor_map']['other']=self.m['prediction']['actor_map'].pop('leader');self.check('LOGICAL_ID_SET')
    def test_future_traits_input(self):
        ref=self.m['source']['artifacts']['prefix']
        self.edit_csv(ref,lambda r:r[-1].update(carla_frame=101,sim_time_s=10.05))
        self.m['prediction']['parent_prefix_sha256']=ref['sha256'];self.m['prediction']['inputs']['prefix']=dict(ref)
        self.check('FUTURE_INFORMATION_IN_PREFIX')
    def test_duplicate_future_frame(self):self.edit_csv(self.m['prediction']['artifacts']['future'],lambda r:r.append(dict(r[-1])));self.check('duplicate')
    def test_missing_future_frame(self):self.edit_csv(self.m['prediction']['artifacts']['future'],lambda r:r.pop());self.check('FUTURE_MISSING')
    def test_future_frame_shift(self):self.edit_csv(self.m['prediction']['artifacts']['future'],lambda r:r[0].update(prediction_frame=1002));self.check('frame/time origin')
    def test_map_mismatch(self):self.edit_json(self.m['prediction']['artifacts']['environment'],lambda d:d.update(map='other'));self.check('ENVIRONMENT_MISMATCH')
    def test_dt_mismatch(self):self.m['prediction']['dt_s']=.1;self.check('DT_MISMATCH')
    def test_bbox_mismatch(self):
        self.edit_json(self.m['prediction']['artifacts']['bbox'],lambda d:d['leader']['extent'].update(x=2.1))
        self.assertAlmostEqual(self.check()['initial_configuration_differences']['bbox'][0]['delta'],.1)
    def test_physics_mismatch(self):
        self.edit_json(self.m['prediction']['artifacts']['physics'],lambda d:d['actors']['leader'].update(mass=1100))
        self.assertEqual(self.check()['initial_configuration_differences']['static_physics'][0]['delta'],100)
    def test_incomplete_labels_keep_trajectory_only(self):
        self.edit_json(self.m['source']['artifacts']['events'],lambda d:d.update(label_status='partial'))
        r=self.check();self.assertFalse(r['label_contract_complete']);self.assertFalse(r['risk_detection_performance_enabled'])
    def test_risk_request_requires_labels(self):
        self.m['evaluation']['requested_metrics'].append('risk_detection')
        self.edit_json(self.m['source']['artifacts']['events'],lambda d:d.update(label_status='missing'))
        self.check('INCOMPLETE_INDEPENDENT_EVENT_LABELS')
    def test_label_coverage_gap(self):
        self.edit_json(self.m['source']['artifacts']['events'],lambda d:d['labels'][0].update(end_frame=199));self.check('LABEL_COVERAGE')
    def test_split_overlap_windows(self):
        self.edit_json(self.m['registry'],lambda d:d['sources'].append(dict(d['sources'][0],split='train')));self.check('SOURCE_SPLIT_DUPLICATE')
    def test_training_on_evaluation_source(self):
        self.m['prediction']['model_data']['training_source_ids']=['synthetic_source'];self.check('SOURCE_SPLIT_LEAKAGE')
    def test_unapproved_threshold_cannot_enable(self):
        self.m['evaluation']['initial_policy'].update(status='APPROVED',thresholds=dict(position_m=.02),approval_reference='self-approval');self.check('UNSUPPORTED_POLICY_APPROVAL')
    def test_method_comparison_structure_still_not_accuracy(self):
        self.m['evaluation']['purpose']='B_method_forecast';self.m['prediction']['control']['leader']['longitudinal']='TM';self.check()
    def test_T_only_valid_counterfactual(self):
        self.m['evaluation']['purpose']='C_T_counterfactual';self.m['prediction']['control']['leader']['parameters']['T_s']=2.
        self.check()
    def test_T_only_rejects_other_parameter_change(self):
        self.m['evaluation']['purpose']='C_T_counterfactual';self.m['prediction']['control']['leader']['parameters'].update(T_s=2.,v0_mps=1.)
        self.check('C_NON_T_CHANGE')
    def test_path_escape(self):self.m['source']['artifacts']['future']['path']='../else.csv';self.check('ARTIFACT_PATH')
    def test_nan(self):self.edit_json(self.m['prediction']['artifacts']['bbox'],lambda d:d['leader']['extent'].update(x=float('nan')));self.check('NONFINITE')
    def test_missing_prefix_frame(self):
        ref=self.m['source']['artifacts']['prefix'];self.edit_csv(ref,lambda r:r.pop(0))
        self.m['prediction']['parent_prefix_sha256']=ref['sha256'];self.m['prediction']['inputs']['prefix']=dict(ref)
        self.check('PREFIX_MISSING')
    def test_duplicate_prefix(self):
        ref=self.m['source']['artifacts']['prefix'];self.edit_csv(ref,lambda r:r.append(dict(r[0])))
        self.m['prediction']['parent_prefix_sha256']=ref['sha256'];self.m['prediction']['inputs']['prefix']=dict(ref)
        self.check('PREFIX_DUPLICATE')
    def test_false_maximum_frame(self):self.m['prediction']['inputs']['maximum_source_frame']=99;self.check('INPUT_MAX_FRAME')
    def test_incomplete_input_inventory(self):self.m['prediction']['inputs']['inventory_complete']=False;self.check('UNDECLARED_INPUTS')
    def test_initial_control_mismatch(self):
        self.edit_json(self.m['prediction']['artifacts']['initial_control'],lambda d:d['actors']['leader'].update(throttle=.1))
        self.assertEqual(self.check()['initial_configuration_differences']['actual_control'][0]['delta'],.1)
    def test_raw_event_bad_time(self):
        self.edit_json(self.m['source']['artifacts']['events'],lambda d:d['raw_events'].append(dict(event_id='e1',kind='collision',frame=101,
            simulation_time_s=10.1,pair=['leader','follower'],provenance='synthetic',raw_payload=dict(impulse=1))))
        self.check('RAW_EVENT_TIME')
    def test_proxy_labels_not_independent(self):
        self.m['evaluation']['requested_metrics'].append('risk_detection')
        self.edit_json(self.m['source']['artifacts']['events'],lambda d:d.update(label_status='proxy'))
        self.check('INCOMPLETE_INDEPENDENT_EVENT_LABELS')
    def test_nonfinite_future_heading(self):
        self.edit_csv(self.m['prediction']['artifacts']['future'],lambda r:r[0].update(yaw='NaN'));self.check('NONFINITE')
    def test_source_hash_alias_between_splits(self):
        self.edit_json(self.m['registry'],lambda d:d['sources'].append(dict(d['sources'][0],source_run_id='alias',split='train')));self.check('SOURCE_HASH_ALIAS')
    def test_prefix_inconsistent_speed(self):
        ref=self.m['source']['artifacts']['prefix'];self.edit_csv(ref,lambda r:r[0].update(speed_mps=99))
        self.m['prediction']['parent_prefix_sha256']=ref['sha256'];self.m['prediction']['inputs']['prefix']=dict(ref)
        self.check('PREFIX_SPEED_DEFINITION')
    def test_collision_label_contradiction(self):
        self.edit_json(self.m['source']['artifacts']['events'],lambda d:d['raw_events'].append(dict(event_id='e1',kind='collision',frame=101,
            simulation_time_s=10.05,pair=['leader','follower'],provenance='synthetic',raw_payload=dict(impulse=1))))
        self.check('COLLISION_LABEL_CONTRADICTION')
    def test_invalid_T_counterfactual(self):
        self.m['evaluation']['purpose']='C_T_counterfactual';self.m['prediction']['control']['leader']['parameters']['T_s']=-1
        self.check('C_T_VALUE')
    def test_measured_clock_mismatch(self):self.edit_csv(self.m['prediction']['artifacts']['clock'],lambda r:r[1].update(simulation_time_s=100.1));self.check('CLOCK_FRAME_TIME')
    def test_clock_missing(self):self.edit_csv(self.m['prediction']['artifacts']['clock'],lambda r:r.pop());self.check('CLOCK_MISSING')
    def test_client_server_version_mismatch(self):self.edit_json(self.m['prediction']['artifacts']['environment'],lambda d:d.update(server_version='0.9.15'));self.check('CARLA_VERSION_MISMATCH')
    def test_complete_synthetic_labels_cannot_enable_scoring(self):
        self.m['evaluation']['requested_metrics'].append('risk_detection')
        r=self.check();self.assertTrue(r['label_contract_complete']);self.assertFalse(r['risk_detection_performance_enabled'])
    def test_residual_magnitude_never_decides_acceptance(self):
        ref=self.m['prediction']['artifacts']['snapshot']
        for difference in (0.,1e-8,.001,.02,1.,100.):
            with self.subTest(difference=difference):
                self.edit_json(ref,lambda d:d['actors'][0].update(x=20.+difference))
                result=self.check()
                self.assertAlmostEqual(result['initial_residuals']['leader']['position_xyz_m'],difference)
                self.assertEqual(result['initial_state_assessment'],'NOT_EVALUATED_PENDING_POLICY')
    def test_residual_does_not_mask_causal_rejection(self):
        self.edit_json(self.m['prediction']['artifacts']['snapshot'],lambda d:d['actors'][0].update(x=21.))
        ref=self.m['source']['artifacts']['prefix'];self.edit_csv(ref,lambda r:r[-1].update(carla_frame=101,sim_time_s=10.05))
        self.m['prediction']['parent_prefix_sha256']=ref['sha256'];self.m['prediction']['inputs']['prefix']=dict(ref)
        result=self.check('FUTURE_INFORMATION_IN_PREFIX')
        self.assertEqual(result['initial_residuals']['leader']['position_xyz_m'],1.)
    def test_duplicate_json_key(self):
        self.path.write_text('{"schema_version":1,"schema_version":2}',encoding='utf-8')
        result=validate(self.path)
        self.assertEqual(result['status'],'INVALID');self.assertIn('DUPLICATE_JSON_KEY',result['errors'][0])
    def test_cli_preserves_existing_output(self):
        from Validate_DT_GroundTruth import main
        out=Path(self.temp.name)/'result.json';out.write_text('keep',encoding='utf-8')
        with self.assertRaises(SystemExit):main(['--manifest',str(self.path),'--output',str(out)])
        self.assertEqual(out.read_text(),'keep')
    def test_cli_invalid_manifest_saved_without_correction(self):
        from Validate_DT_GroundTruth import main
        self.path.write_text('{bad JSON',encoding='utf-8');out=Path(self.temp.name)/'result.json'
        self.assertEqual(main(['--manifest',str(self.path),'--output',str(out)]),2)
        self.assertEqual(json.loads(out.read_text())['status'],'INVALID')
        self.assertEqual(self.path.read_text(),'{bad JSON')
    def test_fresh_cli_process_no_carla_import(self):
        out=Path(self.temp.name)/'result.json'
        script="""import builtins,sys
original=builtins.__import__
def guard(name,*args,**kwargs):
    if name=='carla':raise AssertionError('CARLA import forbidden')
    return original(name,*args,**kwargs)
builtins.__import__=guard
from Validate_DT_GroundTruth import main
raise SystemExit(main(sys.argv[1:]))
"""
        env=dict(os.environ,PYTHONPATH=str(Path(__file__).resolve().parents[1]))
        result=subprocess.run([sys.executable,'-X','utf8','-c',script,'--manifest',str(self.path),'--output',str(out)],
                              capture_output=True,text=True,timeout=15,env=env)
        self.assertEqual(result.returncode,0,result.stderr)
        record=json.loads(out.read_text());self.assertEqual(record['status'],'CONTRACT_VALID_PENDING_APPROVAL')
        self.assertFalse(record['independent_forecast_accuracy_enabled'])
if __name__=='__main__':unittest.main()
