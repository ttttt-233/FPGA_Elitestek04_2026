"""RAW1 integer replay ABI v1. No OpenCV or floating point image operations.

Every 3x3 window is causal and unflushed, exactly as in the streaming RTL.
JPEG compression is applied only after RGB565 reproduction and graph overlay.
"""
import numpy as np

# Copied verbatim from Color_Tone_Map.gamma_lut; validated against RTL in sim.
GAMMA = None  # populated below from the fixed ABI table


def expand565(rgb):
    v = np.asarray(rgb, dtype=np.uint16)
    r, g, b = (v >> 11) & 31, (v >> 5) & 63, v & 31
    return np.stack(((r << 3) | (r >> 2), (g << 2) | (g >> 4), (b << 3) | (b >> 2)), -1).astype(np.uint8)


def gray565(v):
    a = np.asarray(v, dtype=np.uint16)
    return ((a >> 3) << 11) | ((a >> 2) << 5) | (a >> 3)


def gray(rgb):
    a = expand565(rgb).astype(np.uint32)
    return ((77*a[..., 0] + 150*a[..., 1] + 29*a[..., 2]) >> 8).astype(np.int32)


def isp(raw, metadata, *, tone=True):
    """MT9M001_ISP: truncated bilinear center, black top/left two pixels.

    Sensor mirror flags describe sensor readout, and must NOT rotate the CFA
    again. BAYER_PATTERN describes the first actually captured sample.
    """
    a = np.asarray(raw, dtype=np.int32)
    h, w = a.shape
    if h < 3 or w < 3:
        raise ValueError('RAW geometry too small')
    tl, tm, tr = a[:-2, :-2], a[:-2, 1:-1], a[:-2, 2:]
    ml, mm, mr = a[1:-1, :-2], a[1:-1, 1:-1], a[1:-1, 2:]
    bl, bm, br = a[2:, :-2], a[2:, 1:-1], a[2:, 2:]
    diag, cross = (tl+tr+bl+br) >> 2, (tm+ml+mr+bm) >> 2
    horizontal, vertical = (ml+mr) >> 1, (tm+bm) >> 1
    yy, xx = np.indices((h-2, w-2))
    phase = (((yy & 1) << 1) | (xx & 1)) ^ 3 ^ metadata['bayer_pattern']
    out = np.zeros((h, w, 3), dtype=np.int32)
    out[2:, 2:, 0] = np.choose(phase, (diag, vertical, horizontal, mm))
    out[2:, 2:, 1] = np.choose(phase, (cross, mm, mm, cross))
    out[2:, 2:, 2] = np.choose(phase, (mm, horizontal, vertical, diag))
    if tone:
        gains = np.array(metadata['gains_q8'], dtype=np.int64)
        out = np.minimum((np.maximum(out-metadata['black_level'], 0)*gains+128) >> 8, 255)
        if metadata['gamma']:
            out = GAMMA[out]
    out = out.astype(np.uint16)
    return (out[..., 0] >> 3) << 11 | (out[..., 1] >> 2) << 5 | (out[..., 2] >> 3)


def center(a):
    b = np.zeros_like(a)
    b[2:, 2:] = a[1:-1, 1:-1]
    return b


def frontend(rgb, metadata):
    """RGB565 -> median/Gaussian -> signed11 Sobel -> integer axis -> NMS."""
    h, w = rgb.shape
    g = gray(rgb)
    windows = np.lib.stride_tricks.sliding_window_view(g, (3, 3))
    filtered = np.zeros((h, w), dtype=np.int32)
    if metadata['filter']:
        filtered[2:, 2:] = (windows*np.array(((1,2,1),(2,4,2),(1,2,1)))).sum((-2,-1)) >> 4
    else:
        filtered[2:, 2:] = np.partition(windows.reshape(h-2, w-2, 9), 4, axis=-1)[..., 4]
    a,b,c = filtered[:-2, :-2],filtered[:-2, 1:-1],filtered[:-2, 2:]
    d,f = filtered[1:-1, :-2],filtered[1:-1, 2:]
    g0,h0,i = filtered[2:, :-2],filtered[2:, 1:-1],filtered[2:, 2:]
    gx, gy = c+2*f+i-a-2*d-g0, g0+2*h0+i-a-2*b-c
    ax, ay = np.abs(gx), np.abs(gy)
    m = np.zeros((h, w), dtype=np.int32)
    m[2:, 2:] = (123*np.maximum(ax, ay)+51*np.minimum(ax, ay)) >> 7
    direction = np.zeros((h, w), dtype=np.uint8)
    direction[2:, 2:] = np.where((ay << 16) < ax*27146, 0,
        np.where((ax << 16) < ay*27146, 2, np.where((gx < 0) != (gy < 0), 3, 1)))
    direction[2:, 2:][(ax == 0) & (ay == 0)] = 0
    before = np.stack((m[1:-1, :-2],m[:-2, :-2],m[:-2, 1:-1],m[:-2, 2:]))
    after = np.stack((m[1:-1, 2:],m[2:, 2:],m[2:, 1:-1],m[2:, :-2]))
    axis = direction[1:-1, 1:-1][None, ...]
    neg = np.take_along_axis(before, axis, axis=0)[0]
    pos = np.take_along_axis(after, axis, axis=0)[0]
    nms = np.zeros_like(m)
    mid = m[1:-1, 1:-1]
    nms[2:, 2:] = np.where((mid > neg) & (mid >= pos), mid, 0)
    nms[:6, :] = 0;nms[:, :6] = 0
    sobel = center(m);sobel[:5, :] = 0;sobel[:, :5] = 0
    mag = nms if metadata['algorithm'] else sobel
    classes = np.where(mag >= metadata['high'], 2,
                       np.where((mag > 0) & (mag >= metadata['low']) & bool(metadata['algorithm']), 1, 0)).astype(np.uint8)
    return classes, center(center(center(rgb))), center(center(filtered)), center(direction), mag


def components(classes, minimum, budget):
    """Bounded seed-area semantics of Canny_Hysteresis_CCL (8-connectivity).

    An untracked provisional seed preserves its entire strong component.
    Components without a strong sample never survive. No float flood-fill.
    """
    labels = np.zeros(classes.shape, dtype=np.int32)
    parent, area, strong, outside = [0], [0], [False], [False]
    def root(k):
        while parent[k] != k:
            parent[k] = parent[parent[k]];k = parent[k]
        return k
    height,width = classes.shape
    for y,x in np.argwhere(classes):
        roots = set()
        if x: roots.add(root(int(labels[y,x-1])))
        if y:
            for xx in range(max(0,x-1), min(width,x+2)):
                roots.add(root(int(labels[y-1,xx])))
        roots.discard(0)
        if not roots:
            r = len(parent)
            # Production CCL has a bounded provisional-label table.
            if r > ((width+1)//2)*((height+1)//2):
                raise ValueError('CCL label capacity exceeded')
            parent.append(r);area.append(0);strong.append(False);outside.append(r > budget)
        else:
            r = min(roots)
            for k in roots-{r}:
                parent[k] = r;area[r] += area[k];strong[r] |= strong[k];outside[r] |= outside[k]
        labels[y,x] = r;area[r] += 1;strong[r] |= classes[y,x] == 2
    keep = np.zeros(len(parent), dtype=np.bool_)
    for k in range(1,len(parent)):
        r = root(k)
        keep[k] = strong[r] and (not minimum or area[r] >= minimum or outside[r])
    return keep[labels]


def process(raw, metadata):
    color = isp(raw, metadata)
    cls, aligned, filtered, directions, _ = frontend(color, metadata)
    edges = components(cls, metadata['min_area'], metadata['area_budget'])
    mode = metadata['mode']
    binary = np.where(edges, 65535, 0).astype(np.uint16)
    if mode == 0: out = aligned
    elif mode == 1: out = gray565(filtered)
    elif mode == 2: out = binary
    elif mode in (3,5):
        out = aligned.copy() if mode == 3 else gray565(gray(aligned))
        out[:,out.shape[1]//2:] = binary[:,out.shape[1]//2:]
    elif mode == 4: out = np.where(edges, 0xf800, aligned)
    elif mode == 6: out = np.where(edges, np.array((0xf800,0x07e0,0x001f,0xffe0))[directions], aligned)
    else: raise ValueError('unsupported replay mode')
    return color, np.asarray(out,dtype=np.uint16)


def overlay(rgb888, metadata):
    """Shape_Labeled_Overlay bbox and 8x8 glyphs, two pixels per font row.

    Saved geometry is authoritative: temporal tracker history is not guessed
    from one photograph. No host-side reclassification changes the labels.
    """
    out = rgb888.copy();height,width = out.shape[:2]
    shapes = metadata['shapes'] if metadata['shape_enable'] else []
    colors = {1:(0,255,0),2:(0,255,255),3:(255,255,0),4:(255,128,255)}
    for shape in reversed(shapes):
        l,t,r,b = shape['bbox'];kind=shape['kind']
        if not kind: continue
        yy,xx=np.ogrid[:height,:width]
        border=(xx>=l)&(xx<=r)&(yy>=t)&(yy<=b)&((xx-l<2)|(r-xx<2)|(yy-t<2)|(b-yy<2))
        out[border]=colors.get(kind,colors[4])
    for shape in reversed(shapes):
        l,t,_,_ = shape['bbox'];kind=shape['kind']
        if not kind: continue
        tx=min(l,width-64);ty=t-16 if t>=16 else t
        name={1:'RECT',2:'CIRC',3:'TRI '}.get(kind,chr(48+shape['digit'])+'   ')
        label=f"#{shape['id']:02X} {name}"[:8].ljust(8)
        for dy in range(16):
            for dx in range(64):
                x,y=tx+dx,ty+dy
                if 0<=x<width and 0<=y<height:
                    font=GLYPHS.get(label[dx//8],0)
                    out[y,x]=colors.get(kind,colors[4]) if (font>>(63-(dy//2)*8-dx%8))&1 else (16,24,32)
    return out


# ABI_CONSTANTS_BEGIN
GAMMA = np.array([0, 13, 22, 28, 34, 38, 42, 46, 50, 53, 56, 59, 61, 64, 66, 69, 71, 73, 75, 77, 79, 81, 83, 85, 86, 88, 90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 106, 108, 109, 110, 112, 113, 114, 115, 117, 118, 119, 120, 121, 122, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 148, 149, 150, 151, 152, 153, 154, 155, 155, 156, 157, 158, 159, 159, 160, 161, 162, 163, 163, 164, 165, 166, 167, 167, 168, 169, 170, 170, 171, 172, 173, 173, 174, 175, 175, 176, 177, 178, 178, 179, 180, 180, 181, 182, 182, 183, 184, 185, 185, 186, 187, 187, 188, 189, 189, 190, 190, 191, 192, 192, 193, 194, 194, 195, 196, 196, 197, 197, 198, 199, 199, 200, 200, 201, 202, 202, 203, 203, 204, 205, 205, 206, 206, 207, 208, 208, 209, 209, 210, 210, 211, 212, 212, 213, 213, 214, 214, 215, 215, 216, 216, 217, 218, 218, 219, 219, 220, 220, 221, 221, 222, 222, 223, 223, 224, 224, 225, 226, 226, 227, 227, 228, 228, 229, 229, 230, 230, 231, 231, 232, 232, 233, 233, 234, 234, 235, 235, 236, 236, 237, 237, 238, 238, 238, 239, 239, 240, 240, 241, 241, 242, 242, 243, 243, 244, 244, 245, 245, 246, 246, 246, 247, 247, 248, 248, 249, 249, 250, 250, 251, 251, 251, 252, 252, 253, 253, 254, 254, 255, 255], dtype=np.uint16)
GLYPHS = {'A': 1739588466116805120, 'C': 4342103635438746624, 'D': 8666124435314735104, 'B': 8953792110620998656, 'F': 9097342149686476800, '#': 2604345179727209472, 'E': 9097342149686492672, 'G': 4342103695601843200, 'I': 4330237533479255040, 'L': 4629771061636922880, 'N': 4783476233180692992, 'R': 8953792110721778176, 'T': 9086038739982489600, '/': 145249953336262656, '0': 4343240608341965824, '1': 1745171347368607232, '2': 4342035242110975488, '3': 4342035310058814464, '4': 870360504579916800, '5': 9097407595358075904, '6': 2026690734748482560, '7': 9079824231408078848, '8': 4342105817315687424, '9': 4342105824827815936}
# ABI_CONSTANTS_END
