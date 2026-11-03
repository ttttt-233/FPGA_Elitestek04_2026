"""Run with Efinity's Python: independently inspect generated image headers.

Uses the installed Programmer's parser/device database without copying vendor
code into the repository. Payloads and padding are compared byte for byte.
"""
from pathlib import Path
import argparse,hashlib,json,os,re,sys

def hex_bytes(path):
    text=Path(path).read_text(encoding='ascii')
    if not re.fullmatch(r'(?:[0-9a-fA-F]{2}\s+)+',text):
        raise ValueError(f'Not an Efinity byte-per-line HEX: {path}')
    return bytes.fromhex(text)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('spec',type=Path)
    args=ap.parse_args()
    spec=json.loads(args.spec.read_text())
    sys.path.insert(0,str(Path(os.environ['EFINITY_HOME'])/'pgm/bin'))
    from efx_pgm.util import bitstream_util as util
    from efx_pgm.opt_reg_processor import OptionRegisterProcessor
    db=util.get_device_db()
    combined=hex_bytes(spec['combined'])
    if len(combined)>spec['flash_bytes']:raise ValueError('Combined image exceeds flash')
    last=spec['images'][-1]
    # The vendor points unused selectors at the next erased 4 KiB sector,
    # and pads the last populated image to that boundary.
    unused_address=(last['address']+len(hex_bytes(last['path']))+4095)//4096*4096
    previous_end=0;results=[]
    for index,item in enumerate(spec['images']):
        original=hex_bytes(item['path']);offset=item['address']
        if len(original)>item['slot_bytes'] or offset<previous_end:raise ValueError('Image overlap/slot overflow')
        if combined[previous_end:offset]!=b'\xff'*(offset-previous_end):raise ValueError('Non-erased gap padding')
        actual=combined[offset:offset+len(original)]
        if len(actual)!=len(original):raise ValueError('Truncated image')
        extracted=args.spec.parent/f'verify_image_{index}.hex'
        extracted.write_text(''.join(f'{v:02X}\n' for v in actual),encoding='ascii',newline='\n')
        source=OptionRegisterProcessor(item['path'],None,db)
        output=OptionRegisterProcessor(str(extracted),None,db)
        if source.die.get('name')!=spec['device'] or output.die.get('name')!=spec['device']:
            raise ValueError('Wrong FPGA device')
        if any(util.get_display_name_from_bitstream(path,processor.die)!=spec['package']
               for path,processor in [(item['path'],source),(str(extracted),output)]):
            raise ValueError('Wrong FPGA package')
        end=util.get_opt_reg_end_loc_from_bitstream(item['path'],source.die)
        start=end-source.opt_length//8
        if actual[:start]!=original[:start] or actual[end:]!=original[end:]:
            raise ValueError('Configuration payload changed outside option register')
        features={f.get('id'):f for f in source.opt_reg.findall('feature')}
        expected={'EN_RUPD':1,'EN_CB':0,'CB_ADDR_1':spec['images'][1]['address'],
                  'CB_ADDR_2':unused_address,'CB_ADDR_3':unused_address}
        allowed=set();observed={}
        for name,value in expected.items():
            feature=features[name];address=int(feature.get('address'));length=int(feature.get('length','1'))
            allowed.update(range(address,address+length))
            bits=output.get_feature_bits(address,length)
            observed[name]=sum(int(bit)<<n for n,bit in enumerate(bits))
            if observed[name]!=value:raise ValueError(f'{name}={observed[name]}, expected {value}')
        if any(a!=b and n not in allowed for n,(a,b) in enumerate(zip(source.opt_bits,output.opt_bits))):
            raise ValueError('Unrelated option register feature changed')
        results.append(dict(profile=item['profile'],cbsel=index,address=offset,bytes=len(actual),
            payload_sha256=hashlib.sha256(actual[end:]).hexdigest(),options=observed))
        previous_end=offset+len(original)
    if len(combined)!=unused_address or combined[previous_end:]!=b'\xff'*(unused_address-previous_end):
        raise ValueError('Invalid trailing sector padding')
    report=dict(status='passed',binary_bytes=len(combined),binary_sha256=hashlib.sha256(combined).hexdigest(),images=results)
    Path(spec['report']).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
