"""Lossless SNAP RAW1 acquisition, DNG archive and deterministic two-JPEG export.

RAW1 header is 128 bytes, little endian. Shapes occupy 6 packed 51-bit slots
at byte40; digit/quality/ID lanes begin at 80/84/88. Bayer is the first sensor
sample phase, before ISP. Sensor mirror flags are descriptive, not a rotation.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import struct
import tempfile

import numpy as np
from PIL import Image

try:
    from . import digital_twin as twin
except ImportError:
    import digital_twin as twin

PRIVATE_PREFIX = b'Ti60Competition\0'
CFA = ((2,1,1,0),(1,2,0,1),(1,0,2,1),(0,1,1,2))
ABI = 'ti60-raw1-integer-v1'


def parse_header(header):
    if len(header)!=128 or header[:4]!=b'RAW1': raise ValueError('not a RAW1 photo')
    u16=lambda pos:struct.unpack_from('<H',header,pos)[0]
    u32=lambda pos:struct.unpack_from('<I',header,pos)[0]
    if u16(4)!=1: raise ValueError('unsupported RAW1 schema')
    width,height=u16(6),u16(8)
    if (width,height) not in ((1280,720),(800,480)) and not (8<=width<=1280 and 8<=height<=720):
        raise ValueError('invalid RAW geometry')
    if header[10]>3 or header[11]!=8 or not 1<=header[36]<=6 or u32(96)!=width*height:
        raise ValueError('invalid RAW layout')
    if header[20]>1 or header[21]>6 or header[22]>1 or header[23]>1 or not 0<u16(26)<u16(24)<=4095:
        raise ValueError('invalid replay controls')
    boxes=int.from_bytes(header[40:80],'little');digits=int.from_bytes(header[80:84],'little')
    qualities=int.from_bytes(header[84:88],'little');ids=int.from_bytes(header[88:96],'little')
    shapes=[]
    for slot in range(header[36]):
        box=(boxes>>(slot*51))&((1<<51)-1)
        shapes.append(dict(slot=slot,kind=box>>48,id=(ids>>(slot*8))&255,
            digit=(digits>>(slot*4))&15,quality=(qualities>>(slot*2))&3,
            bbox=[(box>>36)&4095,(box>>24)&4095,(box>>12)&4095,box&4095]))
    return dict(abi=ABI,width=width,height=height,bayer_pattern=header[10],bits=header[11],
        gains_q8=[u16(12),u16(14),u16(16)],black_level=u16(18),gamma=header[20],mode=header[21],
        algorithm=header[22],filter=header[23],high=u16(24),low=u16(26),
        min_area=u16(28),area_budget=u16(30),frame_id=u32(32),shape_enable=bool(header[37]),
        sensor_mirrors=header[38],shapes=shapes,profile_id=u32(100),
        exposure_lines=u32(104),analog_gain_register=u32(108),header_hex=header.hex())


def _dng_bytes(raw, metadata):
    """Classic little-endian TIFF/DNG 1.4, one uncompressed 8-bit CFA strip.

    DNGPrivateData follows Adobe's null-terminated manufacturer prefix rule.
    No EXIF maker-note offsets or dependencies on adjacent output files.
    """
    h,w=raw.shape
    private=PRIVATE_PREFIX+json.dumps(metadata,ensure_ascii=True,sort_keys=True,separators=(',',':')).encode()
    tags=[]
    def tag(code,kind,values):
        if kind in (1,2,7): data=values if isinstance(values,bytes) else bytes(values);count=len(data)
        elif kind in (3,4):
            values=list(values);count=len(values);data=struct.pack('<'+('H' if kind==3 else 'I')*count,*values)
        elif kind in (5,10):
            values=list(values);count=len(values);data=b''.join(struct.pack('<'+('II' if kind==5 else 'ii'),*pair) for pair in values)
        tags.append((code,kind,count,data))
    text=lambda v:v.encode('ascii')+b'\0'
    tag(254,4,[0]);tag(256,4,[w]);tag(257,4,[h]);tag(258,3,[8]);tag(259,3,[1]);tag(262,3,[32803])
    tag(271,2,text('Ti60Competition'));tag(272,2,text('MT9M001-8bit'))
    tag(273,4,[0]);tag(274,3,[1]);tag(277,3,[1]);tag(278,4,[h]);tag(279,4,[raw.size])
    tag(284,3,[1]);tag(305,2,text(ABI));tag(339,3,[1]);tag(33421,3,[2,2]);tag(33422,1,CFA[metadata['bayer_pattern']])
    tag(50706,1,[1,4,0,0]);tag(50707,1,[1,3,0,0]);tag(50708,2,text('Ti60Competition MT9M001 8bit RAW'))
    tag(50710,1,[0,1,2]);tag(50711,3,[1]);tag(50713,3,[1,1]);tag(50714,5,[(metadata['black_level'],1)])
    tag(50717,4,[255]);tag(50718,5,[(1,1),(1,1)]);tag(50719,4,[0,0]);tag(50720,4,[w,h])
    # A declared working-space approximation, not a measured camera profile.
    matrix=(32404542,-15371385,-4985314,-9692660,18760108,415560,556434,-2040259,10572252)
    tag(50721,10,[(v,10000000) for v in matrix])
    tag(50728,5,[(256,max(1,int(g))) for g in metadata['gains_q8']])
    tag(50740,1,private);tag(50741,3,[1]);tag(50778,3,[21]);tag(50829,4,[0,0,h,w])
    tags.sort()
    extra_start=8+2+12*len(tags)+4
    extra=bytearray();entries=[]
    for code,kind,count,data in tags:
        if len(data)<=4: value=data.ljust(4,b'\0')
        else:
            if len(extra)%2:extra.append(0)
            value=struct.pack('<I',extra_start+len(extra));extra.extend(data)
        entries.append((code,struct.pack('<HHI',code,kind,count)+value))
    if len(extra)%2:extra.append(0)
    raw_offset=extra_start+len(extra)
    entries=[struct.pack('<HHII',273,4,1,raw_offset) if code==273 else entry for code,entry in entries]
    return b'II*\0'+struct.pack('<I',8)+struct.pack('<H',len(tags))+b''.join(entries)+b'\0'*4+extra+raw.tobytes()


def read_dng(path):
    """Read this ABI's DNG and validate RAW/hash/header independently of JPEG."""
    data=Path(path).read_bytes()
    if data[:4]!=b'II*\0': raise ValueError('unsupported TIFF byte order')
    off=struct.unpack_from('<I',data,4)[0];count=struct.unpack_from('<H',data,off)[0]
    tags={}
    sizes={1:1,2:1,3:2,4:4,5:8,7:1,10:8}
    for n in range(count):
        pos=off+2+n*12;code,kind,length=struct.unpack_from('<HHI',data,pos)
        size=sizes[kind]*length
        start=pos+8 if size<=4 else struct.unpack_from('<I',data,pos+8)[0]
        if start+size>len(data): raise ValueError('truncated TIFF tag')
        tags[code]=data[start:start+size]
    private=tags[50740]
    if not private.startswith(PRIVATE_PREFIX): raise ValueError('unknown DNG private ABI')
    metadata=json.loads(private[len(PRIVATE_PREFIX):])
    canonical=parse_header(bytes.fromhex(metadata['header_hex']))
    for key,value in canonical.items():
        if metadata.get(key)!=value: raise ValueError(f'inconsistent private field {key}')
    offset=struct.unpack('<I',tags[273])[0];size=struct.unpack('<I',tags[279])[0]
    raw=data[offset:offset+size]
    if len(raw)!=metadata['width']*metadata['height'] or hashlib.sha256(raw).hexdigest()!=metadata['raw_sha256']:
        raise ValueError('RAW payload integrity mismatch')
    if tags[33422]!=bytes(CFA[metadata['bayer_pattern']]): raise ValueError('CFA mismatch')
    for code,expected in ((256,struct.pack('<I',metadata['width'])),(257,struct.pack('<I',metadata['height'])),
                          (258,b'\x08\0'),(259,b'\x01\0'),(262,struct.pack('<H',32803)),(50717,struct.pack('<I',255))):
        if tags[code]!=expected: raise ValueError(f'inconsistent DNG tag {code}')
    return np.frombuffer(raw,dtype=np.uint8).reshape(metadata['height'],metadata['width']),metadata


def save_photo(raw_bytes, header, stem, *, expected=None):
    metadata=parse_header(header)
    if len(raw_bytes)!=metadata['width']*metadata['height']: raise ValueError('incomplete RAW plane')
    raw=np.frombuffer(raw_bytes,dtype=np.uint8).reshape(metadata['height'],metadata['width'])
    color,processed=twin.process(raw,metadata)
    plain=twin.expand565(color)
    rendered=twin.overlay(twin.expand565(processed),metadata)
    metadata.update(raw_sha256=hashlib.sha256(raw_bytes).hexdigest(),
        color_rgb565_sha256=hashlib.sha256(color.astype('<u2').tobytes()).hexdigest(),
        processed_rgb565_sha256=hashlib.sha256(processed.astype('<u2').tobytes()).hexdigest(),
        overlay_rgb_sha256=hashlib.sha256(rendered.tobytes()).hexdigest(),
        saved_utc=datetime.now(timezone.utc).isoformat(),
        color_matrix_note='XYZ to linear sRGB working approximation; not a measured MT9M001 profile',
        jpeg=dict(quality=95,subsampling=0),graphics='Shape_Labeled_Overlay-v1; saved per-frame detections; no HUD or temporal trail')
    metadata['integer_recipe']=dict(gamma_lut=twin.GAMMA.tolist(),gray_weights=[77,150,29],gray_shift=8,
        magnitude_weights=[123,51],magnitude_shift=7,direction_tangent=[27146,65536],
        causal_windows=True,flush_right_bottom=False,rgb565_quantization=[5,6,5])
    if expected is not None:
        for key in ('color_rgb565_sha256','processed_rgb565_sha256','overlay_rgb_sha256'):
            if metadata[key]!=expected.get(key): raise ValueError(f'replay model/output mismatch: {key}')
    stem=Path(stem)
    if stem.suffix.lower() in ('.dng','.jpg','.jpeg'):stem=stem.with_suffix('')
    paths=[stem.parent/(stem.name+'.dng'),stem.parent/(stem.name+'_bayer.jpg'),stem.parent/(stem.name+'_fpga.jpg')]
    if any(p.exists() for p in paths): raise FileExistsError('output name exists; choose a new photo name')
    stem.parent.mkdir(parents=True,exist_ok=True)
    published=[]
    try:
        with tempfile.TemporaryDirectory(prefix='.raw-photo-',dir=stem.parent) as folder:
            tmp=[Path(folder)/p.name for p in paths]
            tmp[0].write_bytes(_dng_bytes(raw,metadata))
            Image.fromarray(plain).save(tmp[1],quality=95,subsampling=0)
            Image.fromarray(rendered).save(tmp[2],quality=95,subsampling=0)
            check,_=read_dng(tmp[0])
            if not np.array_equal(check,raw): raise ValueError('DNG write verification failed')
            for p in tmp[1:]:
                with Image.open(p) as img:
                    img.load()
                    if img.size!=(metadata['width'],metadata['height']): raise ValueError('JPEG dimensions mismatch')
            for source,dest in zip(tmp,paths):
                # rename refuses a concurrently created destination on Windows.
                source.rename(dest);published.append(dest)
    except Exception:
        for p in published:p.unlink()
        raise
    return [str(p) for p in paths]


def replay_dng(path,stem):
    raw,metadata=read_dng(path)
    return save_photo(raw.tobytes(),bytes.fromhex(metadata['header_hex']),stem,expected=metadata)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dng',type=Path);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    print('\n'.join(replay_dng(args.dng,args.out)))
