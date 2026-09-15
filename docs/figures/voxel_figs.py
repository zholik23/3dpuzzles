# Three figures for the voxel logic: column parity, the prefix sum, and the
# constant-time query. Numbers are computed, not typed.
import io, json, math, os

# ============================================================ 1. column parity
CROSS = [0.5, 3.8, 6.1, 8.2]          # where the skewer pierces the surface
NZ    = 10                             # voxels in the column

def filled_rows():
    """Exactly the rule in MaterialField.cpp: a voxel is solid when its CENTRE
    lies in a span, so k0 = ceil(z0 - 0.5) and k1 = floor(z1 - 0.5)."""
    out = set()
    for p in range(0, len(CROSS) - 1, 2):
        k0 = max(0, math.ceil(CROSS[p] - 0.5))
        k1 = min(NZ - 1, math.floor(CROSS[p + 1] - 0.5))
        for k in range(k0, k1 + 1):
            out.add(k)
    return out

def fig_column():
    solid = filled_rows()
    cw, ch = 62, 40                   # one voxel on screen
    ox, oy = 470, 60                  # top-left of the column
    H = NZ * ch

    p = ['<svg viewBox="0 0 900 690" role="img" aria-label="A vertical column '
         'of voxels through the model. The surface is pierced four times; the '
         'voxels between the first and second crossing and between the third '
         'and fourth are marked solid.">']
    p.append('<defs><marker id="va" viewBox="0 0 10 10" refX="9" refY="5" '
             'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
             '<path d="M 0 0 L 10 5 L 0 10 z" fill="var(--ink-soft)"/></marker></defs>')

    # ---- the shape the skewer passes through: two lumps with a gap
    sx, sw = 150, 210
    def zy(z):                        # model z -> screen y
        return oy + H - (z / NZ) * H
    p.append(f'<path d="M{sx} {zy(0.5):.1f} h{sw} v{zy(3.8)-zy(0.5):.1f} h-{sw} Z" '
             'fill="var(--material)" fill-opacity=".30" stroke="var(--material)" '
             'stroke-width="2"/>')
    p.append(f'<path d="M{sx+30} {zy(6.1):.1f} h{sw-60} v{zy(8.2)-zy(6.1):.1f} h-{sw-60} Z" '
             'fill="var(--material)" fill-opacity=".30" stroke="var(--material)" '
             'stroke-width="2"/>')
    p.append(f'<text class="vlb" x="{sx}" y="{oy-24}">the model, in section</text>')

    # ---- the skewer
    skew = sx + sw * 0.55
    p.append(f'<line x1="{skew:.0f}" y1="{oy+H+34}" x2="{skew:.0f}" y2="{oy-8}" '
             'stroke="var(--cage)" stroke-width="2.5" marker-end="url(#va)"/>')
    p.append(f'<text class="vcage" x="{skew:.0f}" y="{oy+H+54}" text-anchor="middle">the skewer, up Z</text>')

    # ---- the column of voxels
    p.append(f'<text class="vlb" x="{ox}" y="{oy-24}">that column of voxels</text>')
    for k in range(NZ):
        y = oy + H - (k + 1) * ch
        on = k in solid
        p.append(f'<rect x="{ox}" y="{y}" width="{cw}" height="{ch}" '
                 f'fill="var(--material)" fill-opacity="{".55" if on else "0"}" '
                 'stroke="var(--rule)" stroke-width="1"/>')
        # The digit sits to the right so the voxel CENTRE - the thing the rule
        # actually tests - stays visible rather than hiding under it.
        p.append(f'<text class="vnum" x="{ox+cw*0.72}" y="{y+ch/2+6}" '
                 f'text-anchor="middle">{1 if on else 0}</text>')
        p.append(f'<text class="vtag" x="{ox-12}" y="{y+ch/2+5}" text-anchor="end">{k}</text>')
        p.append(f'<circle cx="{ox+cw*0.30}" cy="{y+ch/2}" r="3.4" '
                 f'fill="{"var(--ink)" if on else "var(--ink-soft)"}" opacity=".8"/>')

    # ---- the crossings
    for n, z in enumerate(CROSS):
        y = zy(z)
        entering = (n % 2 == 0)
        col = "var(--cage)" if entering else "var(--reject)"
        p.append(f'<line x1="{sx-18}" y1="{y:.1f}" x2="{ox+cw+130}" y2="{y:.1f}" '
                 f'stroke="{col}" stroke-width="1.6" stroke-dasharray="6 4" opacity=".85"/>')
        p.append(f'<circle cx="{skew:.0f}" cy="{y:.1f}" r="5" fill="{col}"/>')
        lab = "ENTER" if entering else "EXIT"
        p.append(f'<text class="{"vcage" if entering else "vrej"}" '
                 f'x="{ox+cw+140}" y="{y+5:.1f}">{lab}  z = {z}</text>')

    p.append(f'<text class="vlb2" x="{sx-18}" y="{oy+H+92}">Sort the crossings. '
             'You enter the solid at the first, leave at the second,</text>')
    p.append(f'<text class="vlb2" x="{sx-18}" y="{oy+H+116}">enter at the third '
             '\u2014 so the material is the spans between pairs.</text>')
    p.append(f'<text class="vlb2" x="{sx-18}" y="{oy+H+148}">A voxel is solid when '
             'its CENTRE (the dot) falls in a span.</text>')
    p.append('</svg>')
    return "".join(p)

# ============================================================ 2. prefix sum
RAW = [[1, 0, 0],      # y = 0, bottom row, x = 0..2
       [1, 1, 0],      # y = 1
       [0, 1, 0]]      # y = 2

def prefix(raw):
    n = len(raw)
    S = [[0] * n for _ in range(n)]
    for y in range(n):
        for x in range(n):
            up   = S[y-1][x]   if y else 0
            left = S[y][x-1]   if x else 0
            both = S[y-1][x-1] if (x and y) else 0
            S[y][x] = raw[y][x] + up + left - both
    return S

def fig_prefix():
    S = prefix(RAW)
    n = 3
    cw = 74
    p = ['<svg viewBox="0 0 900 430" role="img" aria-label="A three by three '
         'grid of ones and zeros becomes a grid of running totals, each cell '
         'holding the material below and to the left of it.">']
    p.append('<defs><marker id="pa" viewBox="0 0 10 10" refX="9" refY="5" '
             'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
             '<path d="M 0 0 L 10 5 L 0 10 z" fill="var(--ink-soft)"/></marker></defs>')

    def grid(ox, oy, vals, title, sub, accent):
        q = [f'<text class="vlb" x="{ox}" y="{oy-30}">{title}</text>',
             f'<text class="vtag" x="{ox}" y="{oy-12}">{sub}</text>']
        for y in range(n):
            for x in range(n):
                # y = 0 is the bottom row, so draw it last
                sy = oy + (n - 1 - y) * cw
                v = vals[y][x]
                on = v > 0
                q.append(f'<rect x="{ox+x*cw}" y="{sy}" width="{cw}" height="{cw}" '
                         f'fill="{accent}" fill-opacity="{0.10 + 0.13*min(v,4) if on else 0}" '
                         'stroke="var(--rule)" stroke-width="1.4"/>')
                q.append(f'<text class="vbig" x="{ox+x*cw+cw/2}" y="{sy+cw/2+9}" '
                         f'text-anchor="middle">{v}</text>')
        q.append(f'<line x1="{ox}" y1="{oy+n*cw+8}" x2="{ox+n*cw}" y2="{oy+n*cw+8}" '
                 'stroke="var(--ink-soft)" stroke-width="1.4" marker-end="url(#pa)"/>')
        q.append(f'<text class="vtag" x="{ox+n*cw+12}" y="{oy+n*cw+13}">X</text>')
        q.append(f'<line x1="{ox-8}" y1="{oy+n*cw}" x2="{ox-8}" y2="{oy}" '
                 'stroke="var(--ink-soft)" stroke-width="1.4" marker-end="url(#pa)"/>')
        q.append(f'<text class="vtag" x="{ox-14}" y="{oy-6}" text-anchor="end">Y</text>')
        return "".join(q)

    p.append(grid(60, 70, RAW, "raw voxels", "1 = solid, 0 = empty", "var(--material)"))
    p.append(grid(470, 70, S, "running totals", "everything below and to the left",
                  "var(--cage)"))

    p.append('<g color="var(--ink-soft)"><line x1="300" y1="180" x2="440" y2="180" '
             'stroke="currentColor" stroke-width="2" marker-end="url(#pa)"/></g>')
    p.append('<text class="vtag" x="370" y="170" text-anchor="middle">build once</text>')

    p.append('<text class="vlb" x="60" y="382">here + left + down \u2212 the corner they share</text>')
    p.append('<text class="vlb2" x="60" y="408">Left and down each already count that corner, '
             'so it is added twice and has to come off once.</text>')
    p.append('</svg>')
    return "".join(p)

# ============================================================ 3. O(1) query
def fig_query():
    p = ['<svg viewBox="0 0 900 470" role="img" aria-label="A box drawn in '
         'three dimensions with its eight corners labelled; the material inside '
         'is four corner values added and four subtracted.">']

    # a small axonometric box
    def proj(x, y, z):
        return (150 + x * 150 + z * 66, 300 - y * 130 - z * 46)
    C = {}
    for name, (x, y, z) in {
            "A": (0, 0, 0), "B": (1, 0, 0), "C": (1, 0, 1), "D": (0, 0, 1),
            "E": (0, 1, 0), "F": (1, 1, 0), "G": (1, 1, 1), "H": (0, 1, 1)}.items():
        C[name] = proj(x, y, z)

    def edge(a, b, dashed=False):
        d = ' stroke-dasharray="5 4" opacity=".45"' if dashed else ''
        return (f'<line x1="{C[a][0]:.1f}" y1="{C[a][1]:.1f}" '
                f'x2="{C[b][0]:.1f}" y2="{C[b][1]:.1f}" stroke="var(--cage)" '
                f'stroke-width="2"{d}/>')

    # back edges first
    for a, b in (("D", "C"), ("D", "H"), ("D", "A")):
        p.append(edge(a, b, dashed=True))
    for a, b in (("A", "B"), ("B", "C"), ("C", "G"), ("G", "H"), ("H", "E"),
                 ("E", "A"), ("E", "F"), ("F", "B"), ("F", "G")):
        p.append(edge(a, b))
    p.append(f'<path d="M{C["E"][0]:.1f} {C["E"][1]:.1f} L{C["F"][0]:.1f} {C["F"][1]:.1f} '
             f'L{C["G"][0]:.1f} {C["G"][1]:.1f} L{C["H"][0]:.1f} {C["H"][1]:.1f} Z" '
             'fill="var(--cage)" fill-opacity=".10"/>')

    # sign of each corner in the inclusion-exclusion
    sign = {"G": +1, "H": -1, "F": -1, "C": -1, "E": +1, "D": +1, "B": +1, "A": -1}
    for name, (x, y) in C.items():
        s = sign[name]
        col = "var(--cage)" if s > 0 else "var(--reject)"
        glyph = "+" if s > 0 else "\u2212"
        p.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="13" fill="{col}"/>')
        p.append(f'<text x="{x:.1f}" y="{y+5:.1f}" text-anchor="middle" '
                 'font-family="Consolas, monospace" font-size="15" font-weight="700" '
                 f'fill="#fff">{glyph}</text>')
        # Push the letter away from the cube's centre so it never lands on the
        # marker or on an edge.
        mx = sum(c[0] for c in C.values()) / 8.0
        my = sum(c[1] for c in C.values()) / 8.0
        dx, dy = x - mx, y - my
        L = max(1e-6, (dx * dx + dy * dy) ** 0.5)
        lx, ly = x + dx / L * 30.0, y + dy / L * 30.0
        p.append(f'<text class="vcorner" x="{lx:.1f}" y="{ly+6:.1f}" '
                 f'text-anchor="middle">{name}</text>')

    p.append('<text class="vlb" x="470" y="86">material inside the box =</text>')
    p.append('<text class="vmono" x="470" y="128">+G +E +D +B</text>')
    p.append('<text class="vmono2" x="470" y="160">\u2212H \u2212F \u2212C \u2212A</text>')
    p.append('<text class="vlb2" x="470" y="206">Four corner values added, four taken away.</text>')
    p.append('<text class="vlb2" x="470" y="232">Each is one lookup in the table built once</text>')
    p.append('<text class="vlb2" x="470" y="258">at the start.</text>')
    p.append('<text class="vlb" x="470" y="306">Eight numbers \u2014 whatever the size.</text>')
    p.append('<text class="vlb2" x="470" y="334">A box holding 10 voxels and a box holding</text>')
    p.append('<text class="vlb2" x="470" y="360">a million cost exactly the same.</text>')
    p.append('<text class="vtag" x="470" y="400">(in 2D it is four corners; in 3D, eight)</text>')
    p.append('<text class="vlb2" x="60" y="432">This is why the splitter can afford to score '
             '24 candidate planes on every axis, for every cut.</text>')
    p.append('</svg>')
    return "".join(p)

if __name__ == "__main__":
    TMP = r"C:\Users\Admin\.claude\jobs\d0af9bbd\tmp"
    figs = {"column": fig_column(), "prefix": fig_prefix(), "query": fig_query()}
    io.open(os.path.join(TMP, "voxel_figs.json"), "w", encoding="utf-8").write(json.dumps(figs))
    print("filled rows:", sorted(filled_rows()))
    print("prefix grid (bottom row first):", prefix(RAW))
    print("figures:", ", ".join(figs))
