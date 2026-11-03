"""Read timing and resource results required by profile builds."""
import re

ROW = re.compile(r'^\s+(\w+)\s+(\w+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+\(([RF]-[RF])\)\s*$')

def timing_summary(path):
    """Fail closed if either clock-relationship table is missing or malformed."""
    result = {'setup': {}, 'hold': {}}
    mode = None
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith('Setup (Max) Clock Relationship'):
            mode = 'setup'
        elif line.startswith('Hold (Min) Clock Relationship'):
            mode = 'hold'
        elif line.startswith('NOTE: Values'):
            break
        elif mode:
            match = ROW.fullmatch(line)
            if match:
                launch, capture, constraint, slack, edge = match.groups()
                key = f'{launch}->{capture}:{edge}'
                if key in result[mode]:
                    raise ValueError(f'Duplicate {mode} timing row: {key}')
                result[mode][key] = {'constraint_ns': float(constraint), 'slack_ns': float(slack)}
            elif line.strip() and not line.lstrip().startswith('Launch Clock'):
                raise ValueError(f'Unrecognized {mode} timing row: {line}')
    for mode in ('setup', 'hold'):
        if not result[mode]:
            raise ValueError(f'Missing {mode} timing table')
        result[f'min_{mode}_ns'] = min(v['slack_ns'] for v in result[mode].values())
    result['passes'] = result['min_setup_ns'] >= 0 and result['min_hold_ns'] >= 0
    return result

def routed_resources(path):
    text = path.read_text(encoding='utf-8')
    resources = {}
    for label in ('XLRs', 'Memory Blocks', 'DSP Blocks'):
        m = re.search(rf'^{label}: (\d+) / (\d+) \(([\d.]+)%\)', text, re.M)
        if not m:
            raise ValueError(f'Missing routed resource {label}')
        used, capacity, percent = m.groups()
        resources[label] = dict(used=int(used), capacity=int(capacity),
                                percent=float(percent), free=int(capacity)-int(used))
    packed = {}
    for m in re.finditer(r'^\s*XLRs needed for (.+?): (\d+) / (\d+) \(([\d.]+)%\)', text, re.M):
        packed[m.group(1)] = int(m.group(2))
    if not packed or sum(packed.values()) != resources['XLRs']['used']:
        raise ValueError('XLR packing categories do not reconcile')
    return resources, packed
