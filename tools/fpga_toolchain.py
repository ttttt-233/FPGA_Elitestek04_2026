"""Locate installed FPGA tools without machine-specific paths in runners."""
from pathlib import Path
import argparse
import json
import os
import shutil
import subprocess
import hashlib
import datetime
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def input_fingerprints():
    """Include RTL/IP initialization and constraints, excluding generated work.

    Efinity rewrites XML formatting and run-status attributes. Compare its
    structural configuration so those updates cannot hide a lost source entry.
    """
    extensions = {'.v', '.sv', '.vh', '.svh', '.hex', '.mem', '.mif', '.json', '.xml', '.sdc'}
    paths = [p for folder in ('rtl', 'ip') for p in (ROOT/folder).rglob('*')
             if p.is_file() and p.suffix.lower() in extensions]
    paths += list(ROOT.glob('*.xml')) + list(ROOT.glob('*.sdc'))
    paths += [ROOT/'Build_Bitstream.cmd', Path(__file__).resolve()]
    volatile = {'last_change', 'last_run_state', 'last_run_flow', 'config_result_in_sync',
                'design_ood', 'place_ood', 'route_ood'}

    def structural(node, root=False):
        return [node.tag, sorted((key, value) for key, value in node.attrib.items()
                                if not (root and key in volatile)),
                (node.text or '').strip(), [structural(child) for child in node]]

    fingerprints = {}
    for path in sorted(set(paths)):
        data = path.read_bytes()
        if path.name == 'MT9M001_HDMI_Featuring.xml':
            data = json.dumps(structural(ET.fromstring(data), True), ensure_ascii=True).encode('ascii')
        fingerprints[path.relative_to(ROOT).as_posix()] = hashlib.sha256(data).hexdigest()
    return fingerprints


def _select(candidates, required, label):
    tried = []
    for candidate in candidates:
        if not candidate:
            continue
        folder = Path(candidate).expanduser().resolve()
        if folder in tried:
            continue
        tried.append(folder)
        if all((folder / name).is_file() for name in required):
            return folder
    raise FileNotFoundError(f'{label} not found; specify its path explicitly. Tried: '
                            + ', '.join(map(str, tried)))


def find_efinity(explicit=None):
    if explicit:
        return _select([explicit], ['bin/efx_run.bat'], 'Efinity')
    runner = shutil.which('efx_run.bat')
    candidates = [os.environ.get('EFINITY_HOME'), Path(runner).parent.parent if runner else None]
    for drive in ['D:', 'E:', 'F:', 'C:']:
        candidates += [drive + '/Efinity2025.1', drive + '/Efinity',
                       drive + '/Software/Elitestek/Efinity']
        candidates += sorted(Path(drive + '/Efinity').glob('20*'), reverse=True)
    return _select(candidates, ['bin/efx_run.bat'], 'Efinity')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['locate', 'build'])
    parser.add_argument('--efinity', type=Path, help='Efinity installation root')
    args = parser.parse_args()
    efinity = find_efinity(args.efinity)
    if args.action == 'locate':
        print(json.dumps({'efinity': str(efinity)}, indent=2))
        return
    env = dict(os.environ)
    # The vendor batch file must initialize its own installation, even when
    # invoked from a console configured for another Efinity version.
    env.pop('EFINITY_HOME', None)
    env.pop('PYTHONHOME', None)
    manifest_path = ROOT/'build/fpga-toolchain-latest.json'
    manifest_path.parent.mkdir(exist_ok=True)
    started = time.time_ns()
    report = {'status': 'running', 'started': datetime.datetime.now().astimezone().isoformat(),
              'efinity': str(efinity), 'inputs': input_fingerprints(),
              'note': 'Project XML hash is structural; only formatting/run-status changes are ignored.'}

    def record():
        manifest_path.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')

    record()
    print(f'Building with {efinity}; input manifest: {manifest_path}', flush=True)
    result = subprocess.run([str(efinity/'bin/efx_run.bat'),
                             'MT9M001_HDMI_Featuring.xml', '--flow', 'compile'], cwd=ROOT, env=env)
    if result.returncode:
        report.update(status='failed', exit_code=result.returncode)
        record()
        print('Build failed; any existing outflow bitstream is NOT certified by this run.', flush=True)
        raise SystemExit(result.returncode)
    after = input_fingerprints()
    changed = [name for name in sorted(set(report['inputs']) | set(after))
               if report['inputs'].get(name) != after.get(name)]
    if changed:
        report.update(status='invalid-input-change', changed_inputs=changed)
        record()
        raise RuntimeError('Inputs changed during build: '+', '.join(changed))
    report['artifacts'] = {}
    for suffix in ['bit', 'hex']:
        artifact = ROOT / f'outflow/MT9M001_HDMI_Featuring.{suffix}'
        if not artifact.is_file() or not artifact.stat().st_size or artifact.stat().st_mtime_ns < started:
            report.update(status='invalid-artifact', invalid_artifact=str(artifact))
            record()
            raise RuntimeError(f'Expected fresh build artifact missing: {artifact}')
        report['artifacts'][artifact.relative_to(ROOT).as_posix()] = {
            'bytes': artifact.stat().st_size, 'sha256': hashlib.sha256(artifact.read_bytes()).hexdigest()}
    report.update(status='passed', completed=datetime.datetime.now().astimezone().isoformat())
    record()
    print('Build passed: outflow/MT9M001_HDMI_Featuring.bit', flush=True)


if __name__ == '__main__':
    main()
