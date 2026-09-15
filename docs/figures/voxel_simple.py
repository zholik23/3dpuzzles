# Three plain figures for the voxel idea: the running total in 1D, why it is
# worth building, and when each step happens. Numbers are computed.
import io, json, os

VALS = [0, 1, 1, 0, 1, 1, 1, 0]        # a single row of voxels
def running(v):
    S = [0]
    for x in v:
        S.append(S[-1] + x)
    return S                            # S[i] = total of the first i cells

QA, QB = 2, 6                           # the query: cells 2,3,4,5

# ============================================================ 1. the idea, in 1D
def fig_1d():
    S = running(VALS)
    cw = 78
    ox, oy = 70, 110
    p = ['<svg viewBox="0 0 900 520" role="img" aria-label="A row of voxels and '
         'its running totals. The material between two points is one subtraction '
         'of the two stored totals at its ends.">']

    p.append(f'<text class="h1" x="{ox}" y="56">Start in one dimension</text>')
    p.append(f'<text class="p" x="{ox}" y="88">One row of voxels. 1 means solid.</text>')

    # the voxels
    for i, v in enumerate(VALS):
        x = ox + i * cw
        p.append(f'<rect x="{x}" y="{oy}" width="{cw}" height="{cw}" '
                 f'fill="var(--material)" fill-opacity="{".55" if v else "0"}" '
                 'stroke="var(--rule)" stroke-width="1.5"/>')
        p.append(f'<text class="num" x="{x+cw/2}" y="{oy+cw/2+11}" text-anchor="middle">{v}</text>')

    ax, bx = ox + QA * cw, ox + QB * cw

    # the question, bracketed under the cells it asks about
    p.append(f'<path d="M{ax} {oy+cw+10} v14 h{bx-ax} v-14" fill="none" '
             'stroke="var(--cage)" stroke-width="2"/>')
    p.append(f'<text class="pc" x="{(ax+bx)/2}" y="{oy+cw+50}" text-anchor="middle">'
             'how much material in here?</text>')

    # the running totals, offset half a cell so they sit on the BOUNDARIES
    p.append(f'<text class="p" x="{ox}" y="286">Add them up as you go. '
             'Each number is the total so far.</text>')
    cy = 336
    for i, s in enumerate(S):
        x = ox + i * cw
        end = i in (QA, QB)                 # the two the answer is read from
        p.append(f'<circle cx="{x}" cy="{cy}" r="21" fill="var(--cage)" '
                 f'fill-opacity="{".40" if end else ".12"}" stroke="var(--cage)" '
                 f'stroke-width="{3.4 if end else 1.6}"/>')
        p.append(f'<text class="numc" x="{x}" y="{cy+7}" text-anchor="middle">{s}</text>')

    # the answer, braced back up to the two totals it is read from
    got = S[QB] - S[QA]
    assert got == sum(VALS[QA:QB])
    p.append(f'<path d="M{ax} {cy+30} v14 h{bx-ax} v-14" fill="none" '
             'stroke="var(--cage)" stroke-width="2"/>')
    p.append(f'<text class="h2" x="{(ax+bx)/2}" y="{cy+72}" text-anchor="middle">'
             f'{S[QB]} − {S[QA]} = {got}</text>')
    p.append(f'<text class="p" x="{ox}" y="444">The total where the span ends, '
             'minus the total where it starts.</text>')
    p.append(f'<text class="p" x="{ox}" y="476">Two numbers, one subtraction — '
             'however wide the span.</text>')
    p.append('</svg>')
    return "".join(p)

# ============================================================ 2. why bother
def fig_why():
    p = ['<svg viewBox="0 0 900 520" role="img" aria-label="Counting every voxel '
         'costs more as the box grows; reading the corners of a running total '
         'always costs eight lookups.">']
    p.append(f'<text class="h1" x="60" y="56">Why build it at all</text>')

    # left: count everything
    ox, oy, c = 60, 100, 21
    p.append(f'<text class="h2b" x="{ox}" y="{oy-14}">count every voxel</text>')
    for i in range(11):
        for j in range(8):
            p.append(f'<rect x="{ox+i*c}" y="{oy+j*c}" width="{c-2}" height="{c-2}" '
                     'fill="var(--material)" fill-opacity=".45" stroke="none"/>')
    p.append(f'<text class="pr" x="{ox}" y="{oy+8*c+34}">one addition per voxel</text>')
    p.append(f'<text class="p" x="{ox}" y="{oy+8*c+62}">the bigger the box,</text>')
    p.append(f'<text class="p" x="{ox}" y="{oy+8*c+88}">the longer it takes</text>')

    # right: read the corners
    bx = 520
    p.append(f'<text class="h2b" x="{bx}" y="{oy-14}">read the corners</text>')
    w, h = 11 * c - 2, 8 * c - 2
    p.append(f'<rect x="{bx}" y="{oy}" width="{w}" height="{h}" fill="var(--cage)" '
             'fill-opacity=".10" stroke="var(--cage)" stroke-width="2"/>')
    for dx in (0, w):
        for dy in (0, h):
            p.append(f'<circle cx="{bx+dx}" cy="{oy+dy}" r="9" fill="var(--cage)"/>')
    p.append(f'<text class="pc" x="{bx}" y="{oy+8*c+34}">eight lookups \u2014 always</text>')
    p.append(f'<text class="p" x="{bx}" y="{oy+8*c+62}">the size of the box</text>')
    p.append(f'<text class="p" x="{bx}" y="{oy+8*c+88}">makes no difference</text>')

    # the numbers
    rows = [("voxels in the box", "10", "1,000", "60,000"),
            ("counting them", "10", "1,000", "60,000"),
            ("reading corners", "8", "8", "8")]
    ty = 396
    for r, row in enumerate(rows):
        y = ty + r * 38
        cls = "tbh" if r == 0 else ("tbg" if r == 2 else "tb")
        p.append(f'<text class="{cls}" x="60" y="{y}">{row[0]}</text>')
        for k in range(1, 4):
            p.append(f'<text class="{cls}" x="{360 + (k-1)*170}" y="{y}" '
                     f'text-anchor="end">{row[k]}</text>')
        if r == 0:
            p.append(f'<line x1="60" y1="{y+10}" x2="700" y2="{y+10}" '
                     'stroke="var(--ink)" stroke-width="1.5"/>')
    p.append('</svg>')
    return "".join(p)

# ============================================================ 3. when it happens
def fig_when():
    p = ['<svg viewBox="0 0 900 470" role="img" aria-label="The grid is built '
         'once, queried many thousands of times while the cuts are chosen, then '
         'thrown away before the pieces are made.">']
    p.append('<defs><marker id="wa" viewBox="0 0 10 10" refX="9" refY="5" '
             'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
             '<path d="M 0 0 L 10 5 L 0 10 z" fill="var(--ink-soft)"/></marker></defs>')
    p.append('<text class="h1" x="60" y="56">When each step happens</text>')

    steps = [("load the model",  "",                "var(--ink-soft)", ""),
             ("fill the voxels", "ONCE",            "var(--material)", "~0.2 s"),
             ("add them up",     "ONCE",            "var(--material)", "the running totals"),
             ("choose the cuts", "THOUSANDS\nOF TIMES", "var(--cage)", "8 lookups each"),
             ("throw the grid away", "",            "var(--ink-soft)", ""),
             ("make the pieces", "",                "var(--ink-soft)", "no voxels involved")]

    bw, gap, oy = 118, 22, 130
    ox = 60
    for i, (name, tag, col, note) in enumerate(steps):
        x = ox + i * (bw + gap)
        strong = tag != ""
        p.append(f'<rect x="{x}" y="{oy}" width="{bw}" height="92" rx="4" '
                 f'fill="{col}" fill-opacity="{".16" if strong else ".07"}" '
                 f'stroke="{col}" stroke-width="{2.2 if strong else 1.4}"/>')
        words = name.split()
        for k, wd in enumerate(words):
            p.append(f'<text class="stp" x="{x+bw/2}" y="{oy+34+k*19}" '
                     f'text-anchor="middle">{wd}</text>')
        if tag:
            for k, line in enumerate(tag.split("\n")):
                p.append(f'<text class="tag" x="{x+bw/2}" y="{oy-30+k*17}" '
                         f'text-anchor="middle" fill="{col}">{line}</text>')
        if note:
            p.append(f'<text class="note" x="{x+bw/2}" y="{oy+118}" '
                     f'text-anchor="middle">{note}</text>')
        if i + 1 < len(steps):
            p.append(f'<line x1="{x+bw+3}" y1="{oy+46}" x2="{x+bw+gap-3}" y2="{oy+46}" '
                     'stroke="var(--ink-soft)" stroke-width="1.6" marker-end="url(#wa)"/>')

    p.append('<text class="h2" x="60" y="330">Built once. Asked millions of questions. '
             'Then gone.</text>')
    p.append('<text class="p" x="60" y="372">The grid decides WHERE to cut.</text>')
    p.append('<text class="p" x="60" y="404">It is never part of a piece \u2014 those come '
             'from the trivariate and the Boolean,</text>')
    p.append('<text class="p" x="60" y="432">so they keep the model\u2019s exact curved '
             'surface, not a stair-stepped one.</text>')
    p.append('</svg>')
    return "".join(p)

if __name__ == "__main__":
    TMP = r"C:\Users\Admin\.claude\jobs\d0af9bbd\tmp"
    figs = {"1d": fig_1d(), "why": fig_why(), "when": fig_when()}
    io.open(os.path.join(TMP, "voxel_simple.json"), "w", encoding="utf-8").write(json.dumps(figs))
    S = running(VALS)
    print("row      :", VALS)
    print("totals   :", S)
    print(f"query {QA}..{QB-1}: {S[QB]} - {S[QA]} = {S[QB]-S[QA]}  (true {sum(VALS[QA:QB])})")
