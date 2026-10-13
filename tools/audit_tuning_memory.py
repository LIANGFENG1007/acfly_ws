#!/usr/bin/env python3
"""Read-only source/evidence audit; writes only generated tuning state, never flight code.
Passing this audit means records are consistent, not that a vehicle is safe to fly.
"""
from pathlib import Path
import argparse, datetime, hashlib, json, re, subprocess
from zoneinfo import ZoneInfo


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def constants(text):
    return {m.group(1): m.group(2).strip() for m in re.finditer(
        r'inline\s+constexpr\s+[^;=]+?\s+(\w+)\s*=\s*([^;]+);', text)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--campaign', default='build/speed-tuning-20261013')
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    root = (workspace / args.campaign).resolve()
    if not (root / 'original_hashes.json').is_file():
        raise SystemExit('原始证据目录缺失：' + str(root) + '；请恢复原始数据，不得推测补造。')
    originals = json.loads((root / 'original_hashes.json').read_text())
    production_changes = []
    count = 0
    for rel, old in originals.items():
        path = workspace / rel
        if path.suffix not in {'.cpp', '.hpp', '.h', '.cc', '.c', '.xml'} and path.name != 'CMakeLists.txt':
            continue
        if any(p in {'log', 'logs', 'build', 'install'} for p in Path(rel).parts):
            continue
        count += 1
        if not path.is_file() or sha(path) != old:
            production_changes.append(rel)
    old = constants((root / 'original_params.hpp').read_text())
    new = constants((workspace / 'src/exploration_planner/include/exploration_planner/params.hpp').read_text())
    fixed = {k for k in old if k.startswith('CORRIDOR_')} | {
        'HOME_APPROACH_V', 'HOME_GOAL_WALL_MARGIN', 'HOME_SLOW_V',
        'VISUAL_DROP_SAMPLE_S', 'VISUAL_DROP_RELEASE_WAIT_S', 'VISUAL_DROP_XY_TOL_M',
        'VISUAL_DROP_DESCEND_MPS', 'VISUAL_DROP_CLIMB_MPS'}
    frozen_changes = {k: {'old': old.get(k), 'current': new.get(k)}
                      for k in sorted(fixed) if old.get(k) != new.get(k)}
    frozen_code_changes = [p for p in production_changes if
                           '/fly_mission/' in p or '/acfly_interfaces/' in p or
                           'corridor_' in Path(p).name]
    evidence = {}
    required = ['NIGHT_RUN.md', 'campaign.py', 'baseline/build/offline_runner',
                'baseline/manifest.json', 'runs/baseline_delayed/metrics.json',
                'runs/baseline_drift/metrics.json', 'runs/baseline_lag15/metrics.json',
                'original_commit.txt', 'original_params.hpp']
    optional = ['response_engine/identity.json', 'response_engine/manifest.json',
                'recent_response_fit.json', 'response_sweep_summary.json',
                'cruise_agent_safeaccel_report.md', 'drop_agent_report.md',
                'safety_agent_report.md', 'applied_changes.json']
    for rel in required + optional:
        path = root / rel
        evidence[rel] = {'exists': path.is_file(), 'required': rel in required}
        if path.is_file():
            evidence[rel].update(sha256=sha(path), bytes=path.stat().st_size)
    baseline = {}
    errors = []
    applied_changes = []
    applied_file = root / 'applied_changes.json'
    if applied_file.is_file():
        records = json.loads(applied_file.read_text())
        for index, record in enumerate(records):
            path = workspace / record['file']
            actual = sha(path) if path.is_file() else None
            match = actual == record.get('production_sha256')
            successor = next((r for r in records[index + 1:] if r['file'] == record['file']), None)
            # Preserve historical hashes when another recorded change touches
            # the same file. Require an explicit, continuous hash chain; only
            # the final record is expected to equal the current file bytes.
            chain_ok = successor is not None and (
                record.get('superseded_by') == successor.get('candidate') and
                successor.get('previous_production_sha256') == record.get('production_sha256'))
            applied_changes.append({**record, 'current_sha256': actual, 'record_matches_source': match,
                                    'historical_hash_chain_verified': chain_ok})
            if successor is not None and not chain_ok:
                errors.append('broken recorded change history: ' + record['file'])
            elif successor is None and not match:
                errors.append('source differs from recorded local change: ' + record['file'] +
                              '; inspect user edits/evidence, do not restore automatically')
    for name, seconds in [('delayed', 79.66), ('drift', 81.16), ('lag15', 96.3)]:
        path = root / f'runs/baseline_{name}/metrics.json'
        if not path.is_file():
            errors.append(f'baseline_{name} missing'); continue
        m = json.loads(path.read_text())
        baseline[name] = {k: m.get(k) for k in ['virtual_seconds','passed','complete','drops',
                                               'min_clearance','max_release_error_m','binary_sha256']}
        if abs(m['virtual_seconds'] - seconds) > 1e-6:
            errors.append(f'baseline_{name} disagrees with documented time')
    folders = 0; keys = set(); missing_hash = 0; malformed = []
    node_source = (root / 'baseline/source_snapshot/src/exploration_planner/src/exploration_planner_node.cpp').read_text()
    declared_parameters = set(re.findall(
        r'(?:dd|di|db|declare_parameter(?:<[^>]+>)?)\(\s*"([^"\n]+)"', node_source))
    unknown_parameters = {}
    for folder in (root / 'runs').iterdir():
        if not folder.is_dir() or not (folder / 'metrics.json').is_file():
            continue
        try:
            m = json.loads((folder / 'metrics.json').read_text())
            cfg = json.loads((folder / 'scenario.json').read_text())
            for name in cfg.get('parameters', {}):
                if name in declared_parameters or name.startswith('test_'):
                    continue
                warning = unknown_parameters.setdefault(name, {'count': 0, 'examples': []})
                warning['count'] += 1
                if len(warning['examples']) < 3:
                    warning['examples'].append(str(folder / 'scenario.json'))
            if not (folder / 'result.json').is_file() or not (folder / 'trace.jsonl').is_file():
                malformed.append(str(folder)); continue
            folders += 1
            binary = m.get('binary_sha256')
            if not binary:
                missing_hash += 1; continue
            keys.add(hashlib.sha256(json.dumps({'binary':binary,'config':cfg},
                     sort_keys=True,separators=(',',':')).encode()).hexdigest())
        except (OSError,ValueError):
            malformed.append(str(folder))
    baseline_exe = root / 'baseline/build/offline_runner'
    if baseline_exe.is_file():
        digest = sha(baseline_exe)
        for name, data in baseline.items():
            if data.get('binary_sha256') != digest:
                errors.append(f'baseline_{name} binary hash mismatch')
        manifest_path = root / 'baseline/manifest.json'
        if manifest_path.is_file() and json.loads(manifest_path.read_text()).get('binary_sha256') != digest:
            errors.append('baseline manifest binary hash mismatch')
    missing = [k for k,v in evidence.items() if v['required'] and not v['exists']]
    errors += ['required evidence missing: '+x for x in missing]
    errors += ['frozen parameter changed: '+x for x in frozen_changes]
    errors += ['frozen code changed: '+x for x in frozen_code_changes]
    git = subprocess.run(['git','status','--short'],cwd=workspace,text=True,capture_output=True)
    result = {
        'updated_at': datetime.datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
        'scope':'Evidence consistency audit only; not a flight-safety certification.',
        'campaign':str(root), 'production_files_compared':count,
        'production_changes_since_baseline':production_changes,
        'recorded_local_changes':applied_changes,
        'production_changes_without_record':[p for p in production_changes if
            p not in {r['file'] for r in applied_changes}],
        'frozen_parameter_changes':frozen_changes,'frozen_code_changes':frozen_code_changes,
        'git_status':git.stdout.splitlines(),
        'baseline_metrics':baseline,'evidence':evidence,
        'saved_result_folders':folders, 'distinct_binary_and_full_config_hashes':len(keys),
        'count_caveat':'Full config hashes include scene metadata; not a claim of independent statistical trials.',
        'records_without_binary_hash':missing_hash, 'incomplete_or_malformed_records':malformed,
        'parameter_names_not_in_production':unknown_parameters,
        'parameter_name_caveat':'Read source declarations before claiming a parameter changed. test_* fields require the exact isolated source. Historical ignored fields are retained as evidence, not silently repaired.',
        'errors':errors,
        'next_action':'Read latest NIGHT_RUN and agent reports; check actual agent/process status; do not infer adoption from test pass.',
    }
    (root / 'current_state.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ['updated_at','production_files_compared',
          'production_changes_since_baseline','frozen_parameter_changes','saved_result_folders',
          'distinct_binary_and_full_config_hashes','records_without_binary_hash','errors']},ensure_ascii=False,indent=2))
    raise SystemExit(1 if errors else 0)


if __name__ == '__main__':
    main()
