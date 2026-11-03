"""Package validated 720p/480p builds as an Efinity Remote Update Flash Image.

This command only creates and verifies files. It does not connect to hardware.
"""
from pathlib import Path
import argparse,datetime,hashlib,json,os,re,shutil,subprocess
from build_profile import ROOT,PROFILES,inputs,digest,load_profile
from fpga_toolchain import find_efinity

def load_layout():
    layout=json.loads((ROOT/'profiles/multiboot.json').read_text())
    if layout['device']!='Ti60_MP' or layout['package']!='Ti60F225' or layout['image_type']!='internal' or layout['power_on_profile']!=0:
        raise ValueError('Expected Ti60 internal remote update, golden profile 0')
    if [i['profile'] for i in layout['images']]!=list(PROFILES):raise ValueError('Profile/image order mismatch')
    end=0
    for n,image in enumerate(layout['images']):
        if image['cbsel']!=n or image['address']%4096 or image['slot_bytes']%4096 or image['slot_bytes']<=0:
            raise ValueError('Invalid selector, sector alignment or slot size')
        if image['address']<end or n==0 and image['address']!=0:raise ValueError('Overlapping slots or missing golden image')
        end=image['address']+image['slot_bytes']
    if end>layout['flash_bytes'] or layout['flash_bytes']!=8*1024*1024:raise ValueError('W25Q64WS capacity exceeded')
    return layout

def source_inputs_match(expected,current,root=ROOT):
    """Accept only proven LF/CRLF differences from the recorded build bytes.

    Git can change text line endings when a release is cloned on another PC.
    The recorded hashes remain authoritative; neither source edits nor missing
    files are accepted. No file or historical build manifest is rewritten.
    """
    if expected.keys()!=current.keys():return False
    for name,recorded in expected.items():
        if current[name]==recorded:continue
        raw=(root/name).read_bytes()
        lf=raw.replace(b'\r\n',b'\n')
        if recorded not in {hashlib.sha256(lf).hexdigest(),
                            hashlib.sha256(lf.replace(b'\n',b'\r\n')).hexdigest()}:
            return False
    return True

def verify_artifact(path,metadata):
    if path.is_file():
        valid=path.stat().st_size==metadata['bytes'] and digest(path)==metadata['sha256']
    elif path.suffix=='.bin' and path.with_suffix('').is_file():
        # Raw companions are Git-ignored. Decode the tracked byte-per-line
        # source and check the exact recorded binary hash without writing it.
        from verify_multiboot_vendor import hex_bytes
        data=hex_bytes(path.with_suffix(''))
        valid=len(data)==metadata['bytes'] and hashlib.sha256(data).hexdigest()==metadata['sha256']
    else:valid=False
    if not valid:raise ValueError('Artifact integrity failure: '+str(path))

def validate_sources(layout):
    current=inputs(ROOT);reports=[]
    for image in layout['images']:
        stem='race4_'+image['profile'].replace('-','_')
        manifest=ROOT/'outflow/profiles'/f'{stem}.manifest.json'
        report=json.loads(manifest.read_text())
        if report['status']!='passed' or not report['timing']['passes']:raise ValueError('Build/timing not passed')
        if report['configuration']!=load_profile(image['profile']) or not source_inputs_match(report['source_inputs'],current):
            raise ValueError('Profile artifacts do not match current configuration/source; rebuild both profiles')
        for name,sha in report['configuration_inputs'].items():
            if digest(ROOT/name)!=sha:raise ValueError('Build configuration fingerprint mismatch: '+name)
        for suffix,metadata in report['artifacts'].items():
            path=manifest.with_name(stem+'.'+suffix)
            verify_artifact(path,metadata)
        reports.append(report)
    return reports

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--efinity',type=Path)
    args=ap.parse_args();layout=load_layout();reports=validate_sources(layout);efinity=find_efinity(args.efinity)
    work=ROOT/'build/multiboot'/datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f');work.mkdir(parents=True)
    combined=work/'race4_dual_profile.hex'
    env=dict(os.environ);env.update(EFINITY_HOME=str(efinity),EFXPGM_HOME=str(efinity/'pgm'),QT_QPA_PLATFORM='offscreen',
        PYTHONHOME=str(efinity/'python311'),EFINITY_USER_DIR=str(work/'efinity-user'))
    env.pop('ENA_BOOT_IMAGE1',None)
    env.pop('EXTRA_SYNC_PATTERN',None)
    python=efinity/'python311/bin/python.exe'
    cmd=[python,efinity/'pgm/bin/multi_image_merger.py','-m','selectable_flash_image','-t','internal']
    spec=dict(layout,combined=str(combined),report=str(work/'header_verification.json'))
    spec['images']=[]
    for image,report in zip(layout['images'],reports):
        stem='race4_'+image['profile'].replace('-','_')
        source=ROOT/'outflow/profiles'/f'{stem}.hex'
        snapshot=work/f'{stem}.hex';shutil.copy2(source,snapshot)
        cmd+=['-ifile',snapshot,'-iaddr',hex(image['address'])]
        spec['images'].append(dict(image,path=str(snapshot)))
        if report['artifacts']['hex.bin']['bytes']>image['slot_bytes']:raise ValueError('Image exceeds reserved slot')
    cmd+=['-o',combined]
    def run(label,command):
        result=subprocess.run(list(map(str,command)),cwd=work,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
            encoding='utf-8',errors='replace',timeout=300)
        (work/(label+'.log')).write_text(result.stdout,encoding='utf-8')
        if result.returncode or re.search(r'(?im)^ERROR[: ]|Traceback',result.stdout):raise RuntimeError(result.stdout[-6000:])
    run('vendor_merge',cmd)
    if not combined.is_file() or not combined.with_suffix('.rpt').is_file():raise RuntimeError('Vendor merger produced no image/report')
    spec_path=work/'verify_spec.json';spec_path.write_text(json.dumps(spec,indent=2)+'\n')
    run('verify_headers',[python,ROOT/'tools/verify_multiboot_vendor.py',spec_path])
    verification=json.loads((work/'header_verification.json').read_text())
    validate_sources(layout)
    manifest=dict(status='passed',completed=datetime.datetime.now().astimezone().isoformat(),layout=layout,
        physical_board_tested=False,flash_programmed=False,work_directory=str(work),
        combined_hex=dict(bytes=combined.stat().st_size,sha256=digest(combined)),verification=verification,
        source_builds={p['configuration']['profile_name']:dict(artifacts=p['artifacts'],
            resources=p['resources'],timing=p['timing']) for p in reports},
        source_inputs=reports[0]['source_inputs'],
        packaging_inputs={p.relative_to(ROOT).as_posix():digest(p) for p in
            [ROOT/'profiles/multiboot.json',Path(__file__),ROOT/'tools/verify_multiboot_vendor.py']})
    destination=ROOT/'outflow/profiles'
    shutil.copy2(combined,destination/combined.name)
    shutil.copy2(combined.with_suffix('.rpt'),destination/'race4_dual_profile.rpt')
    (destination/'race4_dual_profile.manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (work/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(status='passed',file=str(destination/combined.name),hex=manifest['combined_hex'],verification=verification),indent=2))

if __name__=='__main__':main()
