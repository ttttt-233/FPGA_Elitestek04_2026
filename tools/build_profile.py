"""Build common RTL in an isolated, fingerprinted resolution profile snapshot.

No implicit release: canonical outflow/MT9M001_HDMI_Featuring.bit is never touched.
JSON is used for defaults + small overrides; no third-party YAML dependency.
"""
from pathlib import Path
import argparse
import copy
import datetime
import hashlib
import json
import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

from fpga_toolchain import ROOT, find_efinity
from build_reports import timing_summary, routed_resources

DESIGN = 'MT9M001_HDMI_Featuring'
PROFILES = ('720p-quality', '480p-60')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def merge(base, overrides):
    result = copy.deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict):
            result[key] = merge(result.get(key, {}), value)
        else:
            result[key] = value
    return result


def load_profile(name):
    if name not in PROFILES:
        raise ValueError(f'Unknown profile: {name}')
    cfg = merge(json.loads((ROOT/'profiles/defaults.json').read_text()),
                json.loads((ROOT/f'profiles/resolution/{name}.json').read_text()))
    cfg['profile_name'] = name
    cfg['reconfiguration'] = dict(interface='Titanium internal reconfiguration',
        enabled=True,cbsel=cfg['profile_id'],power_on_profile=0)
    w, h, s, d, pll = cfg['width'], cfg['height'], cfg['sensor'], cfg['display'], cfg['pll']
    if cfg['profile_id'] != PROFILES.index(name) or cfg['shape_targets'] != 6:
        raise ValueError('Unexpected profile ID/target count')
    if cfg['system_clock_hz'] != 96000000 or cfg['pixel_bits'] != 16:
        raise ValueError('This milestone requires the validated 96 MHz/RGB565 datapath')
    media=cfg['media']
    expected=(8,112,128) if name==PROFILES[0] else (16,256,320)
    if tuple(media[k] for k in ('photo_frames','video_frames','pre_frames'))!=expected or media['fps']!=30:
        raise ValueError('Unexpected profile media capacity/cadence')
    raw_bytes=((w*h+2047)//2048)*2048
    photo_bytes=w*h*2+raw_bytes+2048
    media_end=0x04000000+expected[0]*photo_bytes+(expected[1]+expected[2])*w*h*2
    raw_end=media_end+4*raw_bytes
    if raw_end>0x20000000 or w*h*2+raw_bytes+2048>1<<22:
        raise ValueError('RAW photo allocation exceeds the DDR address range/slot')
    if (w, h) != ((1280,720) if name == PROFILES[0] else (800,480)):
        raise ValueError('Profile geometry mismatch')
    if w % 4 or h % 4 or s['row_start'] % 2 or s['column_start'] % 2:
        raise ValueError('Geometry must preserve the validated Bayer phase and four-pixel grid')
    if s['hblank'] < 19 or s['vblank'] < 15 or s['exposure_lines'] > h+s['vblank']:
        raise ValueError('Invalid sensor blanking or frame-extending exposure')
    if s['row_start']+h > 1048 or s['column_start']+w > 1312:
        raise ValueError('Sensor window exceeds array')
    vco = cfg['system_clock_hz'] * pll['sys_feedback_divider']
    if vco / pll['pixel_divider'] != d['pixel_clock_hz']:
        raise ValueError('HDMI clock and PLL disagree')
    if pll['pixel_divider'] != 2*pll['pixel_2x_divider'] or pll['pixel_divider'] != 10*pll['pixel_10x_divider']:
        raise ValueError('HDMI serializer clock ratios disagree')
    if abs(1344000000/pll['camera_divider']-s['clock_hz']) > 1:
        raise ValueError('Sensor clock and PLL disagree')
    ht = w + sum(d[f'h_{k}'] for k in ('front','sync','back'))
    vt = h + sum(d[f'v_{k}'] for k in ('front','sync','back'))
    camera_clocks = (w+244+s['hblank']-19)*(h+s['vblank']+1)
    cfg['derived'] = dict(frame_pixels=w*h, frame_bytes=w*h*cfg['pixel_bits']//8,
        grid_width=(w+3)//4,grid_height=(h+3)//4,h_total=ht,v_total=vt,media_end_exclusive=media_end,
        raw_capture_end_exclusive=raw_end,raw_plane_bytes=raw_bytes,photo_bytes=photo_bytes,
        hdmi_calculated_hz=d['pixel_clock_hz']/ht/vt,
        camera_frame_clocks=camera_clocks,camera_calculated_fps=s['clock_hz']/camera_clocks,
        publish_budget_cycles=cfg['system_clock_hz']//cfg['build_fps'])
    if name == '480p-60' and (cfg['derived']['camera_calculated_fps'] < 60 or abs(cfg['derived']['hdmi_calculated_hz']-60) > .1):
        raise ValueError('480p clock budget does not meet 60 Hz target')
    return cfg


def top_parameters(cfg):
    s,d=cfg['sensor'],cfg['display']
    result=dict(BUILD_PROFILE_ID=cfg['profile_id'],BUILD_PROFILE_WIDTH=cfg['width'],
        BUILD_PROFILE_HEIGHT=cfg['height'],BUILD_PROFILE_FPS=cfg['build_fps'],
        PROFILE_PIXEL_CLOCK_HZ=d['pixel_clock_hz'],PROFILE_ROW_START=s['row_start'],
        PROFILE_COLUMN_START=s['column_start'],PROFILE_HBLANK=s['hblank'],PROFILE_VBLANK=s['vblank'],
        PROFILE_EXPOSURE_LINES=s['exposure_lines'])
    result.update({f'PROFILE_{key.upper()}':d[key] for key in ('h_front','h_sync','h_back','v_front','v_sync','v_back')})
    result.update(PROFILE_SYSTEM_CLOCK_HZ=cfg['system_clock_hz'],PROFILE_PIXEL_BITS=cfg['pixel_bits'],
        PROFILE_SHAPE_TARGETS=cfg['shape_targets'],PROFILE_ROW_MIRROR=s['row_mirror'],
        PROFILE_COLUMN_MIRROR=s['column_mirror'],PROFILE_BAYER_PATTERN=s['bayer_pattern'])
    result.update({f'PROFILE_{key.upper()}':value for key,value in cfg['isp'].items()})
    result.update({f'PROFILE_{key.upper()}':cfg['media'][key] for key in ('photo_frames','video_frames','pre_frames')})
    return result


def inputs(root):
    paths=[p for folder in ('rtl','ip') for p in (root/folder).rglob('*')
           if p.is_file() and p.suffix.lower() in ('.v','.sv','.vh','.svh','.hex','.mem','.mif','.json','.xml','.sdc')]
    paths += list(root.glob('*.xml'))+list(root.glob('*.sdc'))
    return {p.relative_to(root).as_posix():digest(p) for p in sorted(set(paths))}


def configure(folder,cfg):
    top=folder/'rtl/Board_IO_Top.v'
    source=top.read_text()
    for name,value in top_parameters(cfg).items():
        source,count=re.subn(rf'(parameter integer {name}\s*=\s*)\d+',rf'\g<1>{value}',source)
        if count!=1: raise ValueError(f'Expected unique top parameter {name}')
    top.write_text(source, newline='\n')
    # Patch only output dividers. Pin bindings, DDR clocks and phase settings
    # remain the same. The source tree still has one Interface Designer project.
    path=folder/f'{DESIGN}.peri.xml'
    source=path.read_text()
    for clock,key in [('clk_sys','sys_feedback_divider'),('clk_pixel','pixel_divider'),
                      ('clk_pixel_2x','pixel_2x_divider'),('clk_pixel_10x','pixel_10x_divider'),
                      ('clk_27m','camera_divider')]:
        source,count=re.subn(rf'(<efxpt:comp_output_clock name="{clock}"[^>]*out_divider=")\d+',
                            rf'\g<1>{cfg["pll"][key]}',source)
        if count!=1: raise ValueError(f'Expected unique clock {clock}')
    path.write_text(source,newline='\n')
    # Keep all original CDC/DDR/IO constraints. Replace only HDMI clock periods.
    # For 720p preserve the original bytes and rounding exactly.
    if cfg['profile_id']:
        path=folder/f'{DESIGN}.pt.sdc';source=path.read_text()
        pixel_period=1e9/cfg['display']['pixel_clock_hz']
        for clock,ratio in [('clk_pixel',1),('clk_pixel_2x',2),('clk_pixel_10x',10)]:
            period=pixel_period/ratio
            waveform=f'-waveform {{{period/4:.4f} {period*3/4:.4f}}} ' if ratio==10 else ''
            source,count=re.subn(rf'^create_clock .*\b{clock}$',f'create_clock {waveform}-period {period:.4f} {clock}',source,flags=re.M)
            if count!=1: raise ValueError(f'Expected unique clock constraint {clock}')
        path.write_text(source,newline='\n')


def xml_configuration(path):
    # Efinity writes version/run metadata, which is recorded separately. No
    # source, constraint, device or tool option differences are accepted.
    volatile={'sw_version','last_change','last_run_state','last_run_flow',
              'config_result_in_sync','design_ood','place_ood','route_ood'}
    def node(e,root=False):
        return [e.tag,sorted((k,v) for k,v in e.attrib.items() if not(root and k in volatile)),
                (e.text or '').strip(),[node(c) for c in e]]
    return node(ET.parse(path).getroot(),True)


def validate_profile_warnings(text):
    undriven=[line for line in text.splitlines() if 'profile_' in line and
              re.search(r'does not have a driver|undriven|not driven',line,re.I)]
    if undriven:
        raise RuntimeError('Undriven profile signal; no release artifacts published:\n'+'\n'.join(undriven))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile',choices=PROFILES)
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--resume',type=Path,help='Resume this prepared isolated build, verifying fingerprints')
    parser.add_argument('--efinity',type=Path)
    args=parser.parse_args()
    cfg=load_profile(args.profile)
    profile_root=ROOT/'build/profiles'/args.profile
    if args.resume:
        folder=args.resume.resolve()
        if not folder.is_relative_to(profile_root.resolve()): parser.error('--resume outside profile directory')
        report=json.loads((folder/'manifest.json').read_text())
        if report['configuration']!=cfg or report['prepared_inputs']!=inputs(folder):
            raise RuntimeError('Prepared configuration or inputs changed')
    else:
        folder=profile_root/datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        folder.mkdir(parents=True)
        (folder/'build').mkdir()
        for name in ('rtl','ip'): shutil.copytree(ROOT/name,folder/name)
        for name in (f'{DESIGN}.xml',f'{DESIGN}.peri.xml',f'{DESIGN}.pt.sdc'):
            shutil.copy2(ROOT/name,folder/name)
        configure(folder,cfg)
        report=dict(status='prepared',configuration=cfg,source_inputs=inputs(ROOT),prepared_inputs=inputs(folder),

            configuration_inputs={p.relative_to(ROOT).as_posix():digest(p) for p in
                [ROOT/'profiles/defaults.json',ROOT/f'profiles/resolution/{args.profile}.json',Path(__file__)]},
            measurements=dict(camera_measured_fps=None,hdmi_measured_hz=None,processed_fps=None,
                recognition_publish_fps=None,board_tested=False),
            note='Calculated clock rates are not measured FPS. Six Shape/Digit targets, persistent tracking, outlined trails, gradient overlay and runtime filters enabled.')
    def save():
        (folder/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
        (profile_root/'latest.json').write_text(json.dumps(dict(directory=str(folder),status=report['status']),indent=2)+'\n')
    save();print(f'PROFILE_DIR={folder}',flush=True)
    if args.prepare_only: return
    efinity=find_efinity(args.efinity)
    before_xml=xml_configuration(folder/f'{DESIGN}.xml')
    canonical={p.name:digest(p) for p in (ROOT/'outflow').glob(f'{DESIGN}.*') if p.suffix in ('.bit','.hex')}
    report.update(status='building',started=datetime.datetime.now().astimezone().isoformat(),efinity=str(efinity));save()
    env=dict(os.environ);env.pop('EFINITY_HOME',None);env.pop('PYTHONHOME',None)
    try:
        with (folder/'build.console.log').open('w',encoding='utf-8') as log:
            result=subprocess.run([str(efinity/'bin/efx_run.bat'),f'{DESIGN}.xml','--flow','compile'],
                cwd=folder,env=env,stdout=log,stderr=subprocess.STDOUT)
        if result.returncode: raise RuntimeError(f'Efinity exited {result.returncode}; see {folder}/build.console.log')
        after=inputs(folder)
        changed=[k for k in set(after)|set(report['prepared_inputs']) if after.get(k)!=report['prepared_inputs'].get(k)]
        if any(k!=f'{DESIGN}.xml' for k in changed) or xml_configuration(folder/f'{DESIGN}.xml')!=before_xml:
            raise RuntimeError(f'Build input changed: {changed}')
        report['tool_xml_metadata_changes']=changed
        validate_profile_warnings((folder/'outflow'/f'{DESIGN}.warn.log').read_text(encoding='utf-8',errors='replace'))
        report['profile_driver_check']='passed'
        report['timing']=timing_summary(folder/'outflow'/f'{DESIGN}.timing.rpt')
        report['resources'],report['xlr_packing']=routed_resources(folder/'outflow'/f'{DESIGN}.route.rpt')
        if not report['timing']['passes']: raise RuntimeError('Setup/hold failed; no release artifacts published')
        if canonical!={p.name:digest(p) for p in (ROOT/'outflow').glob(f'{DESIGN}.*') if p.suffix in ('.bit','.hex')}:
            raise RuntimeError('Canonical artifacts changed during profile build')
        report['artifacts']={}
        dest=ROOT/'outflow/profiles';dest.mkdir(exist_ok=True)
        stem='race4_'+args.profile.replace('-','_')
        for suffix in ('bit','hex','bit.bin','hex.bin'):
            p=folder/'outflow'/f'{DESIGN}.{suffix}'
            if not p.is_file() or not p.stat().st_size: raise RuntimeError(f'Missing artifact {p}')
            report['artifacts'][suffix]=dict(bytes=p.stat().st_size,sha256=digest(p))
            shutil.copy2(p,dest/f'{stem}.{suffix}')
        report.update(status='passed',completed=datetime.datetime.now().astimezone().isoformat());save()
        (dest/f'{stem}.manifest.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(dict(profile=args.profile,status=report['status'],resources=report['resources'],
            min_setup_ns=report['timing']['min_setup_ns'],min_hold_ns=report['timing']['min_hold_ns'],artifacts=report['artifacts']),indent=2),flush=True)
    except Exception as error:
        report.update(status='failed',error=str(error));save();raise


if __name__=='__main__': main()
