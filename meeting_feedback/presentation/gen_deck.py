import io, json, os, datetime

# Styled after the lab template ppt_tmpl_v2025.2: white slides, a blue-to-green
# gradient bar across the top carrying the topic (left) and the slide number
# (right), a bold black title under it, a Korean gothic face.
ROOT = os.path.dirname(os.path.abspath(__file__))
SL = os.path.join(ROOT, "project", "slides")
os.makedirs(SL, exist_ok=True)

INK = "#1D1D1F"; PAPER = "#FFFFFF"; BODY = "#3A4048"; MUTED = "#5B636B"
P1 = "#0F6FC6"; P2 = "#0C7CAE"; P3 = "#00B050"; TEAL = "#009DD9"; HL = "#F49100"
CARD = "#F2F7FB"; RULE = "#D5DEE6"
BAR = f"linear-gradient(90deg, {P1} 0%, {P2} 50%, {P3} 100%)"
# The old names, kept so the slide content below reads unchanged:
ORANGE = P2      # emphasis text, step numbers
BLUE = P3
SERIF = "'Noto Sans KR', Arial, sans-serif"
SANS = "'Noto Sans KR', Arial, sans-serif"
MONO = "'IBM Plex Mono', 'Courier New', monospace"
ACCENTS = {P2: P1, P3: P3, INK: TEAL}   # card rule colours: blue, green, teal

slides = []   # (id, html); {NUM} is filled in from the final order


def bar(topic):
    return (f'<div style="position:absolute; left:0px; top:0px; width:1920px; height:84px; background:{BAR}"></div>'
            f'<p style="position:absolute; left:53px; top:18px; width:1300px; font-size:32px; font-weight:500; color:#FFFFFF">{topic}</p>'
            f'<p style="position:absolute; right:53px; top:18px; width:200px; text-align:right; font-size:32px; font-weight:500; color:#FFFFFF">{{NUM}}</p>')


def title(t, color=INK):
    return f'<h2 style="font-family:{SANS}; font-size:52px; font-weight:700; line-height:1.2; color:{color}">{t}</h2>'


def card(head, body, flex="1", accent=None, size=28):
    edge = f"border-top:6px solid {ACCENTS.get(accent, accent)}" if accent else f"border:1px solid {RULE}"
    return (f'<div style="flex:{flex}; display:flex; flex-direction:column; gap:16px; background:{CARD}; padding:40px; {edge}; border-radius:8px">'
            f'<h3 style="font-family:{SANS}; font-size:34px; font-weight:700; line-height:1.25; color:{INK}">{head}</h3>'
            f'<p style="font-size:{size}px; line-height:1.5; color:{BODY}">{body}</p></div>')


def shot(label, w="100%", h="100%", flex="1"):
    return (f'<div style="flex:{flex}; display:flex; flex-direction:column; justify-content:center; align-items:center; background:#F5F7F9; border:2px dashed #9AA6B2; border-radius:8px; padding:40px">'
            f'<p style="font-size:28px; line-height:1.4; text-align:center; color:{MUTED}">{label}</p></div>')


def formula(t, size=32):
    return f'<p style="font-family:{MONO}; font-size:{size}px; line-height:1.4; color:{INK}; background:#EAF3FA; padding:20px 28px; border-radius:8px">{t}</p>'


def bullets(items, size=28):
    lis = "".join(f"<li>{i}</li>" for i in items)
    return f'<ul style="font-size:{size}px; line-height:1.5; color:{BODY}; display:flex; flex-direction:column; gap:14px">{lis}</ul>'


def table(head, widths, rows, size=26, width=1664, pad=None):
    th = "".join(f'<th style="width:{w}%; text-align:left; color:#FFFFFF' + (f'; padding:{pad}' if pad and n == 0 else '') + f'">{h}</th>' for n, (h, w) in enumerate(zip(head, widths)))
    trs = [f'<tr style="background:{P2}">{th}</tr>']
    for i, r in enumerate(rows):
        shade = ' style="background:#F2F7FB"' if i % 2 else ""
        trs.append(f"<tr{shade}>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>")
    return f'<table style="width:{width}px; font-size:{size}px; color:{BODY}; font-family:{SANS}">{"".join(trs)}</table>'


def content(sid, topic, ttl, body, notes, section, transition="fade"):
    html = (f'<section id="{sid}" data-transition="{transition}" style="background:{PAPER}; color:{BODY}; font-family:{SANS}; '
            f'padding:124px 128px 96px; display:flex; flex-direction:column; gap:36px">'
            f'{bar(topic)}{title(ttl)}{body}<aside>{notes}</aside></section>')
    slides.append((sid, html))


def dark(sid, topic, big, sub, notes):
    # The statement slide: the template's gradient, full bleed.
    html = (f'<section id="{sid}" data-transition="fade" style="background:{BAR}; color:#FFFFFF; font-family:{SANS}; '
            f'padding:128px; display:flex; flex-direction:column; justify-content:center; gap:48px">'
            f'<p style="font-size:32px; font-weight:500; color:#FFFFFF">{topic}</p>'
            f'<h1 style="font-family:{SANS}; font-size:84px; font-weight:700; line-height:1.2; color:#FFFFFF; width:1500px">{big}</h1>'
            f'<p style="font-size:36px; line-height:1.45; color:#FFFFFF; width:1400px">{sub}</p>'
            f'<aside>{notes}</aside></section>')
    slides.append((sid, html))


row = lambda *items, gap=32: f'<div style="display:flex; gap:{gap}px">{"".join(items)}</div>'
col = lambda *items, flex="1", gap=24: f'<div style="flex:{flex}; display:flex; flex-direction:column; gap:{gap}px">{"".join(items)}</div>'
p = lambda t, size=28, color=BODY: f'<p style="font-size:{size}px; line-height:1.5; color:{color}">{t}</p>'

S1, S2, S3, S4, S5 = "The question", "How it works", "Results", "Elber §6 and open questions", "Next and Q&amp;A"

# ---------------------------------------------------------------- 1 cover
# The template's title slide: the bar, a centred title, the lab and name lines,
# and a light grey triangle in the bottom-right corner.
slides.append(("cover", f'''<section id="cover" data-transition="fade" style="background:{PAPER}; color:{INK}; font-family:{SANS}; padding:128px; display:flex; flex-direction:column; justify-content:center; align-items:center; gap:56px">
<div style="position:absolute; left:0px; top:0px; width:1920px; height:84px; background:{BAR}"></div>
<div style="position:absolute; left:1669px; top:829px; width:251px; height:251px"><svg aria-label="" width="251" height="251" viewBox="0 0 251 251" xmlns="http://www.w3.org/2000/svg"><polygon points="251,0 251,251 0,251" fill="#F2F2F2"/></svg></div>
<div style="display:flex; flex-direction:column; align-items:center; gap:24px">
<h1 style="font-family:{SANS}; font-size:96px; font-weight:700; line-height:1.15; color:{INK}; text-align:center">Interlocking by curvature</h1>
<p style="font-size:36px; line-height:1.4; color:{BODY}; text-align:center; width:1400px">Cutting Elber's volumetric puzzles in the parameter domain: can a curved interface lock the pieces by itself?</p>
</div>
<div style="display:flex; flex-direction:column; align-items:center; gap:12px">
<p style="font-size:36px; color:{INK}; text-align:center">그래픽스 및 기하처리 연구실</p>
<p style="font-size:36px; color:{INK}; text-align:center">정보컴퓨터공학부 예기즈바에브 졸란</p>
</div>
<aside>Goal of the talk: report the first experiment from last meeting's plan. Reproduce Elber's 2x2x2 without pins, build a translational directional blocking graph (DBG) with exact normals, vary R keeping det J positive, and tabulate blocking. Short answer: curvature does block, but only through the thickness, and no valid case has a single key. The DBG tells us exactly why, and that points at the trivariate, not at the joints.</aside>
</section>'''))

# ---------------------------------------------------------------- 2 reframe
content("reframe", "Where we left off", "The feedback, in three lines",
        row(card("Diagnosis", "A planar BSP in R³ makes the trivariate irrelevant, and a planar partition always comes apart. Dovetails can go anywhere, so there is no principled place for them.", accent=ORANGE),
            card("Reframing", "Cut in the parameter domain D instead. A plane in D is mapped by M to a curved surface in R³, and a curved interface can refuse a pull.", accent=BLUE),
            card("Test", "Build a translational DBG with exact normals n′ ∝ J⁻ᵀn. Check for a single key, then a level k. Keep det J of M∘R positive.", accent=INK))
        + p("First experiment: Elber's 2×2×2 puzzle with the pins removed, vary R, and tabulate blocked pairs, immobile pieces and the single key.", 28, INK),
        "This is what the professor said last time, in my words. The current app cut with planes in R³ using a bounding-box trivariate, so the map was affine and the trivariate did nothing. A planar partition always comes apart because every pair can slide along its separating plane's normal. The proposal: cut in D, let M bend the cut, and test if the bend alone blocks. The harmonic field is for building the trivariate (Martin, Cohen and Kirby 2009), not for cutting.",
        S1)

# ---------------------------------------------------------------- 3 dmap
svg_dmap = f'''<svg aria-label="A square domain D divided by straight cuts is mapped by M to a bent shape where the same cuts are curved" width="1664" height="440" viewBox="0 0 1664 440" xmlns="http://www.w3.org/2000/svg">
<rect x="120" y="40" width="360" height="360" fill="{CARD}" stroke="{INK}" stroke-width="3"/>
<line x1="120" y1="220" x2="480" y2="220" stroke="{INK}" stroke-width="3"/>
<line x1="300" y1="40" x2="300" y2="400" stroke="{HL}" stroke-width="8"/>
<line x1="600" y1="220" x2="880" y2="220" stroke="{INK}" stroke-width="4"/>
<polygon points="880,200 920,220 880,240" fill="{INK}"/>
<path d="M 1060 70 C 1200 20, 1400 40, 1520 110 C 1590 210, 1580 320, 1500 400 C 1340 440, 1160 430, 1050 390 C 1000 280, 1010 170, 1060 70 Z" fill="{CARD}" stroke="{INK}" stroke-width="3"/>
<path d="M 1030 235 C 1150 190, 1390 280, 1582 240" fill="none" stroke="{INK}" stroke-width="3"/>
<path d="M 1285 40 C 1345 160, 1235 290, 1310 425" fill="none" stroke="{HL}" stroke-width="8"/>
</svg>'''
dmap_body = (f'<div style="position:relative; width:1664px; height:480px">{svg_dmap}'
             f'<p style="position:absolute; left:0px; top:436px; width:600px; text-align:center; font-size:28px; color:{INK}">Parameter domain D: cuts are planes</p>'
             f'<p style="position:absolute; left:640px; top:150px; width:280px; text-align:center; font-size:36px; font-family:{MONO}; color:{INK}">M</p>'
             f'<p style="position:absolute; left:1000px; top:436px; width:640px; text-align:center; font-size:28px; color:{INK}">R³: the same cuts are curved</p></div>'
             + p("Elber divides D into cells and maps each cell through the trivariate M. When M is affine the cut stays flat and resists nothing. When M bends, the same cut becomes a freeform interface.", 28))
content("dmap", "The idea", "A flat cut in D becomes a curved cut in R³", dmap_body,
        "D is the unit cube [0,1]³ with coordinates u, v, w. M maps it onto the solid. A cell of D is a box; its image under M is a puzzle piece. The orange line is one cut plane, u = 0.5; after M it is a curved sheet. The whole experiment asks whether that curvature is enough to stop pieces sliding apart.",
        S1)

# ---------------------------------------------------------------- 4 statement
content("statement", "Problem statement", "What we are trying to show",
        f'<p style="font-family:{SANS}; font-size:40px; font-weight:500; line-height:1.4; color:{INK}; border-left:6px solid {P2}; padding:8px 0px 8px 40px">Given a closed genus-0 solid, construct a partition into k pieces that is interlocking in R³: no proper subassembly separates by translation except a single designated key.</p>'
        + table(["criterion", "what it asks"], [32, 68],
              [["Interlocking in R³", "A single key first, then a level k"],
               ["Positive Jacobian", "det J of M∘R &gt; 0 everywhere: the map never folds"],
               ["One component, minimum thickness", "Every piece is one solid, thick enough to print"],
               ["Uniform sizes", "Pieces of roughly equal volume"],
               ["No exposed joints", "No joint visible from a cavity (once joints exist)"]], 26),
        "The full statement is 148 words, in problem_statement.md. The five criteria come straight from the professor's list. Every one of them is now measured by the analyser, except joints in cavities, which does not apply yet because this experiment has no joints.",
        S1)

# ---------------------------------------------------------------- 5 setup
content("setup", "Step 1 · Reproduce Elber, pins removed", "Two models from Elber's own script",
        row(col(bullets([
            "Script: Elber's <b>puz_vol.irt</b> (IRIT), copied as puz_vol - Copy.irt with every pin and hole deleted.",
            "<b>PuzzleVolEarth</b>: a sphere shell. <b>PuzzleVolDuck</b>: ruled between the duck surface and an inner core.",
            "Divided 2×2×2 in D: u around the axis, v along the profile, w through the thickness.",
            "Tiles at size 1.0 (Elber's 0.99 left room for pins, so tiles would not touch). The trivariate M is saved next to the pieces.",
        ]), flex="1"), shot("[Screenshot: GuIrit, PuzzleVolEarth and PuzzleVolDuck, pins removed]", flex="1")),
        "Only these two models are used, both from the user's copy of Elber's script. Elber's own grids are N×M×1: he never cuts through the thickness. The professor asked for 2x2x2, which adds the radial cut w; that matters a lot for the results. The trivariate is saved because the analyser needs M itself, not just the tessellated tiles.",
        S1)

# ---------------------------------------------------------------- 6 rparam
content("rparam", "Step 3 · The R sweep", "R is a randomization amount, not a radius",
        row(col(formula("RandomizeInterior(TV, R):<br>interior control point += random(−R, R)"),
                bullets([
                    "Boundary control points untouched, so the outer shape is unchanged; only the inside bends.",
                    "Random generator re-seeded before every R: each R scales one fixed field of offsets. Without that, each R was a different random shape.",
                    "The duck is refined in depth first: with 2 control points across its thickness, R had nothing to move.",
                ]), flex="3"),
            col(table(["model", "valid R", "folds"], [34, 33, 33],
                      [["sphere", "0 to 0.125", "0.15 to 0.5"], ["duck", "0 to 0.015", "0.02 to 0.1"], ["Elber's Earth", "-", "0.2 (his value)"]], 26, 653),
                p("Folded cases are kept on purpose, to show what folding does.", 26), flex="2")),
        "R is only the amplitude of a random perturbation of interior control points; the file names r000, r005 and so on encode R×100 for the sphere and R×1000 for the duck. Re-seeding matters: RandomizeInterior draws fresh randoms per call, so before the fix R=0.05 and R=0.10 were unrelated shapes and the sweep was scatter. Elber's own R=0.2 folds the sphere (interior det J = -0.081), which is fine for him because his puzzle is held by pins.",
        S1)

# ---------------------------------------------------------------- 7 dbgflow
def step(n, head, body):
    return (f'<div style="flex:1; display:flex; flex-direction:column; gap:14px; background:{CARD}; padding:32px; border:1px solid {RULE}; border-radius:8px">'
            f'<p style="font-size:44px; font-weight:700; color:{P2}">{n}</p>'
            f'<h3 style="font-size:30px; font-weight:600; line-height:1.2; color:{INK}">{head}</h3>'
            f'<p style="font-size:24px; line-height:1.4; color:{BODY}">{body}</p></div>')
arrow = '<x-connector style="width:40px; color:#9AA1A8"></x-connector>'
content("dbgflow", "Step 2 · The directional blocking graph", "How the DBG works, in five steps",
        f'<div style="display:flex; gap:8px; align-items:center">{step("1", "Sample each shared face", "11 × 11 points on every interface of the division")}{arrow}'
        f'{step("2", "Exact normals", "n′ = M_a × M_b from the map, not a mesh")}{arrow}'
        f'{step("3", "Exact clearance", "How far a pull may tilt and still come out")}{arrow}'
        f'{step("4", "Test every group", "A group moves if its boundary faces allow it")}{arrow}'
        f'{step("5", "Key, level k, take-apart", "Read the answers off the graph")}</div>'
        + p("Everything is computed from the trivariate M, on the same cells the IRIT script builds. Piece bounding boxes match the IRIT tiles to within 0.01.", 28, INK),
        "This is the outline for the next five slides. The DBG in the literature (Wilson and Latombe; Song 2012) is a graph over pieces whose edges say which directions are blocked by whom. Our version works on freeform interfaces, so each edge carries a set of directions sampled on the sphere, not the six axis directions the old PlannerBlocking class used.",
        S2)

# ---------------------------------------------------------------- 8 test
def normal_arrows(pts):
    return "".join(f'<line x1="{a}" y1="{b}" x2="{c}" y2="{d}" stroke="{P1}" stroke-width="5" marker-end="url(#ah)"/>' for a, b, c, d in pts)
defs = f'<defs><marker id="ah" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="{P1}"/></marker></defs>'
svg_flat = (f'<svg aria-label="Flat interface: all normals point the same way" width="696" height="340" viewBox="0 0 780 380" xmlns="http://www.w3.org/2000/svg">{defs}'
            f'<rect x="0" y="0" width="780" height="230" fill="#E4F4EA"/><rect x="0" y="230" width="780" height="150" fill="#DCE6F0"/>'
            f'<line x1="0" y1="230" x2="780" y2="230" stroke="{INK}" stroke-width="4"/>'
            + normal_arrows([(130, 230, 130, 130), (300, 230, 300, 130), (470, 230, 470, 130), (640, 230, 640, 130)]) + '</svg>')
svg_curved = (f'<svg aria-label="Curved interface: normals fan out" width="696" height="340" viewBox="0 0 780 380" xmlns="http://www.w3.org/2000/svg">{defs}'
              f'<rect x="0" y="0" width="780" height="380" fill="#E4F4EA"/><path d="M 60 380 L 60 330 Q 390 -40 720 330 L 720 380 Z" fill="#DCE6F0"/>'
              f'<path d="M 60 330 Q 390 -40 720 330" fill="none" stroke="{INK}" stroke-width="4"/>'
              + normal_arrows([(126, 263, 66, 196), (258, 175, 221, 93), (390, 145, 390, 55), (522, 175, 559, 93), (654, 263, 714, 196)]) + '</svg>')
def panel(svg, cap, lab_b, lab_a):
    return (f'<div style="flex:1; display:flex; flex-direction:column; gap:16px"><div style="position:relative; width:696px; height:340px">{svg}'
            f'<p style="position:absolute; left:24px; top:16px; width:300px; font-size:26px; font-weight:600; color:{INK}">{lab_b}</p>'
            f'<p style="position:absolute; left:24px; top:285px; width:300px; font-size:26px; font-weight:600; color:{INK}">{lab_a}</p></div>'
            f'<p style="font-size:26px; line-height:1.4; color:{BODY}">{cap}</p></div>')
content("test", "DBG · step 1", "One interface: which pulls does it allow?",
        formula("A may slide along d  ⇔  d · n ≤ 0 at every sample point (n points out of A)", 30)
        + row(panel(svg_flat, "Flat cut: one normal, so exactly half of all directions are free (2000 of 4000).", "piece B", "piece A"),
              panel(svg_curved, "Curved cut: the normals fan out and the allowed set shrinks. Once they spread past 90°, no direction is left: the pair is <b>blocked</b>.", "piece B", "piece A"), gap=40),
        "If A moves along d, a point on the shared face moves into B exactly when d has a positive component along the outward normal there. So A can move only if d points backwards against every normal. A flat face has one normal, so half the sphere of directions works. On a curved face each normal removes a different half-sphere, and if the normals spread wide enough the intersection is empty. Spread here means the widest angle between a normal and the mean normal; 90 degrees is the threshold for a symmetric patch.",
        S2)

# ---------------------------------------------------------------- 9 normals
content("normals", "DBG · step 2", "Normals come from the map, not a mesh",
        row(col(bullets([
            "11 × 11 points on each shared face in D, for example the face w = 0.5 between two cells.",
            "Tangents along the face, M_u and M_v, by central differences (step 10⁻⁵ of the domain).",
            "Normal n′ = M_u × M_v. That is J⁻ᵀn up to scale, with no matrix inverse and no approximation.",
            "Where the map degenerates (the poles of the sphere) the cross product is round-off; points with area under 10⁻⁴ of the face's largest are dropped.",
        ]), formula("n′ ∝ J⁻ᵀ e_w = (M_u × M_v) / det J", 30), flex="3"),
            card("Why not the IRIT piece meshes?", "Triangles give only facet normals and carry no Jacobian, so neither J⁻ᵀn nor det J can be computed from them. The analyser divides the same M the script used.", flex="2", accent=BLUE)),
        "Why the identity holds: the columns of J are M_u, M_v, M_w. The rows of J inverse transpose are the cofactors divided by det J, and the cofactor for the w row is exactly M_u × M_v. So the exact transformed normal is the cross product of the two tangents of the interface, which is how it is computed. The pole filter was a real bug: before it, noise normals at the sphere's poles made pairs look blocked that were free. That is covered on the verification slide.",
        S2)

# ---------------------------------------------------------------- 10 directions
def bignum(n, lab):
    return (f'<div style="flex:1; display:flex; flex-direction:column; gap:8px; background:{CARD}; padding:36px; border:1px solid {RULE}; border-radius:8px">'
            f'<p style="font-size:84px; font-weight:700; line-height:1.1; color:{P2}">{n}</p>'
            f'<p style="font-size:28px; color:{BODY}">{lab}</p></div>')
content("directions", "DBG · step 3", "Blocked or free: the exact clearance",
        formula("clearance = distance( origin, convex hull of the face normals )", 30)
        + row(bignum("90°", "a single flat face: half of all pulls work"),
              bignum("35.3°", "a corner piece of a box: an octant of pulls"),
              bignum("0.19°", "sphere R = 0.05, piece 7: a sliver, but free"))
        + bullets([
            "A group can slide along d only if d points back against every normal on its boundary. Such a d exists exactly when the origin lies outside the convex hull of those normals; the distance is how far the pull may tilt.",
            "Computed with Gilbert's algorithm, which gives a certified lower bound, so 'free' is proven, not sampled. Under 1° is flagged <b>marginal</b>: free in exact geometry, only within manufacturing tolerance.",
            "Why not sample directions: 4000 of them (3° apart) missed piece 7 entirely; 400 000 found 33.",
        ], 26),
        "This replaced the original test, which checked 4000 sampled directions and called a group blocked if none of them worked. That fails on thin free sets: at sphere R = 0.05 piece 7 was reported blocked with 0 of 4000 directions, but it slides out, and 400 000 directions found 33 that free it. It also produced a false single key at sphere R = 0 with 40 000 directions, when one sample landed on the knife-edge line. The clearance is exact over the sampled normals: max over unit d of min over normals of (-d . n), which is the distance from the origin to their convex hull, the sine of the tilt the pull can tolerate. Gilbert's algorithm walks to the nearest hull point; x . v / |x| is a certified lower bound at every step. Checks: a flat face gives 90 degrees, a box corner asin(1/sqrt 3) = 35.26 degrees, both exact. Sampled directions are still used as a certified prefilter: if not one direction clears the normals even with an allowance of the sampling spacing, the group is certainly blocked. Up to 12 pieces everything is decided exactly; larger divisions fall back to sampling and the report says so.",
        S2)

# ---------------------------------------------------------------- 11 groups
content("groups", "DBG · step 4", "Groups, not pieces, decide",
        formula("group S moves along d  ⇔  every face crossing S's boundary allows d", 30)
        + row(card("Duck: piece 0 alone", "It must separate from piece 1 across the curved radial face, whose normals spread 109° to 119°. No direction works. Stuck.", accent=INK),
              card("Duck: pieces {0, 1} together", "The radial face is now inside the group and constrains nothing. Only the flat u and v cuts remain, and about a quarter of all directions is free. Escapes.", accent=ORANGE))
        + p("Every one of the 2ⁿ − 2 groups is tested: 254 for 8 pieces, and a complete search up to 18 pieces (262 142 groups, under 5 seconds).", 28, INK),
        "This is the key distinction between a pile of pairwise facts and a blocking graph. Zero pieces free alone does not mean interlocked: the duck has zero, but every radial pair slides out together. The mobility of a group is the AND of the bit-sets of the faces on its boundary; faces inside the group are skipped. For 8 pieces that is 254 groups; for more than 18 pieces the search is capped and the app says the answer is not a proof.",
        S2)

# ---------------------------------------------------------------- 12 keylevel
content("keylevel", "DBG · step 5", "Single key, level k, and a take-apart",
        row(card("Single key", "Exactly one piece can move, and no other piece or group can. Stricter than 'one piece is free': a pair that can escape disqualifies it.", accent=ORANGE),
            card("Level k (Song 2012)", "Remove the key, test what is left, repeat. k counts the states in a row with a unique key. Chen 2022's level (moves before the first piece comes free) needs motion search and is not computed.", accent=BLUE),
            card("Take-apart", "Repeatedly remove the smallest group that can move. It ends 'stuck' when what is left cannot move, for example a blocked pair.", accent=INK)),
        "The checklist version, 'exactly one piece has a valid path', misses groups. Under either definition there is no valid single key here. Level k: Song, Fu and Cohen-Or 2012 recursive interlocking. Chen et al. 2022 define the level as the number of moves needed before the first piece can be removed, which requires finite motions and a configuration search, not a first-order blocking graph. Which one the professor means is an open question.",
        S2)

# ---------------------------------------------------------------- 13 verify
content("verify", "Is the DBG right?", "Checked by actually sliding the pieces",
        row(col(p("512 points inside the moving pieces are moved along d in 40 steps, up to 40% of the model's size. After each step, Newton's method inverts M to find which piece's cell the point is in. Landing inside a piece that is not moving is a collision. No normals are used.", 28),
                p("Flat control: every face has exactly 90° of clearance, and every corner piece 35.26°, the textbook values.", 28, INK), flex="2"),
            col(table(["case", "group +d", "group −d", "member alone"], [28, 22, 22, 28],
                      [["flat control", "clear, agrees", "collides, agrees", "-"],
                       ["duck R = 0", "clear, agrees", "collides, agrees", "both collide, agrees"],
                       ["sphere R = 0.05", "clear, agrees", "collides, agrees", "-"],
                       ["sphere R = 0", "clear, agrees", "collides, agrees", "outer shell clear: knife edge"]], 24, 979),
                p("13 of 14 checks agree. The exception is explained two slides on.", 26), flex="3")),
        "The DBG is first order: it says the first infinitesimal step is free, and on a flat control a global sign flip would still give half the sphere. The slide check catches both: the group must clear along +d and collide along -d. It found two bugs, now fixed: noise normals at the sphere poles, and the direction set lacking opposites. Limits: 8x8x8 samples can miss a very thin overlap, and points that leave the model are no longer tracked. Translation only: assemblable under translational blocking; rotational or swept check pending.",
        S2)

# ---------------------------------------------------------------- 14 jacobian
content("jacobian", "Step 3 · Validity", "The Jacobian: does the rubber fold?",
        row(col(formula("det J = M_u · (M_v × M_w)", 36),
                bullets([
                    "The local volume stretch of M. Zero or negative: the map folds and the piece passes through itself.",
                    "Central differences on a 13 × 13 × 13 grid over all of D, boundary included, plus 7 × 7 × 7 inside every piece.",
                    "Interior minimum reported separately: det J = 0 at the poles of a revolution is degenerate, not folded.",
                    "Interior det J ≤ 0: the case is invalid and not ranked. The app colours folded pieces red.",
                ], 26), flex="3"),
            col(bignum("0.125", "largest valid R on the sphere (det J 0.28 → 0.08)"),
                bignum("0.015", "largest valid R on the duck (det J 0.036 → 0.002)"), flex="2")),
        "The same tangents used for the normals give the Jacobian columns. Sampling includes the domain boundary because the sphere's poles sit exactly on v = 0 and v = 1; a centre-only grid would report a healthy map. The duck has almost no headroom: its R = 0 Jacobian is 0.036 against the sphere's 0.282, so it folds at R = 0.02. Elber's own R = 0.2 folds the sphere, with interior det J = -0.081 and one folded piece.",
        S2)

# ---------------------------------------------------------------- 15 shape
content("shape", "The other criteria", "Thickness, one component, sizes",
        row(card("Minimum thickness", "From 5 × 5 points on each face, 10% in from its edges, shoot a ray straight inward. Newton tracks it until it leaves the piece's cell; bisection finds the exit. The shortest ray is the thickness.", accent=BLUE),
            card("One component", "True by construction: a piece is one box pushed through a continuous map. It can only fail by folding (det J &lt; 0), which the Jacobian check catches.", accent=INK),
            card("Uniform sizes", "Volume = ∫ det J over the piece's cell (6 × 6 × 6 midpoint rule). Report largest / smallest volume: pass ≤ 2, warn ≤ 3.", accent=ORANGE))
        + p("Joints exposed in cavities: not applicable yet, because this experiment has no joints.", 26),
        "Thickness details: faces that collapse to a point or a curve (the centre of the ball, the duck's tip) are skipped, because their normal is round-off and the ray grazes along a neighbouring cut. The 10% inset exists because material always tapers to nothing at a sharp edge, like the rim of a hemisphere. Limit in the app: a slider in percent of model size, default 3%, which is arbitrary until we have a printing size in mm.",
        S2)

# ---------------------------------------------------------------- 16 headline
dark("headline", "Results",
     "Curvature blocks, but only through the thickness, and never enough for a key.",
     "No valid case has a single key. Level k is 0 everywhere. The DBG shows exactly why.",
     "The one-line summary. Curvature does block: 4 of 12 interfaces on the duck, where a plane blocks none. But on a ruled trivariate over a surface of revolution only one of the three cut families is curved, so every piece has exactly one blocker and pairs escape. The next slides show the table, the mechanism, and what happens past the fold.")

# ---------------------------------------------------------------- 17 results: step 4, the table
def shape_block(name, tbl):
    return (f'<div style="flex:1; display:flex; flex-direction:column; gap:12px">'
            f'<h3 style="font-size:32px; font-weight:700; color:{INK}">{name}</h3>{tbl}</div>')
TBL_HEAD = ["R", "det J", "blocking pairs", "immobilized", "single key"]
TBL_W = [13, 13, 34, 23, 17]
content("results", "Step 4 · The table", "Blocking pairs, immobilized pieces, single key",
        row(shape_block("Sphere (PuzzleVolEarth)", table(TBL_HEAD, TBL_W, [['0', '0.282', '0–1, 2–3, 4–5, 6–7', '8 of 8', 'no'], ['0.025', '0.251', 'none', '4 of 8', 'no'], ['0.05', '0.222', 'none', '4 of 8', 'no'], ['0.075', '0.194', '6–7', '5 of 8', 'no'], ['0.1', '0.168', '6–7', '5 of 8', 'no'], ['0.125', '0.085', '6–7', '5 of 8', 'no'], ['0.15', '-0.092, folds', '-', '-', 'invalid']], 24, 816, "8px 10px")),
            shape_block("Duck (PuzzleVolDuck)", table(TBL_HEAD, TBL_W, [['0', '0.036', '0–1, 2–3, 4–5, 6–7', '8 of 8', 'no'], ['0.0025', '0.036', '0–1, 2–3, 4–5, 6–7', '8 of 8', 'no'], ['0.005', '0.036', '0–1, 2–3, 4–5, 6–7', '8 of 8', 'no'], ['0.0075', '0.035', '0–1, 2–3, 4–5, 6–7', '8 of 8', 'no'], ['0.01', '0.029', '0–1, 2–3, 4–5, 6–7', '8 of 8', 'no'], ['0.015', '0.002', '0–1, 2–3, 4–5, 6–7', '8 of 8', 'no'], ['0.02', '-0.031, folds', '-', '-', 'invalid']], 24, 816, "8px 10px")))
        + p("<b>Blocking pair</b>: no direction separates them. <b>Immobilized</b>: cannot slide out alone. <b>Single key</b>: exactly one piece can move and nothing else can. Decided exactly, by clearance. Pairs 0–1, 2–3, 4–5, 6–7 are the two layers of one quarter (the radial cut). Controls: flat box, 0 of 12 blocked; duck 2×2×1 (Elber's topology), 0 of 4 blocked.", 24),
        "This is the table the professor asked for in step 4, one row per R, for both base shapes. Blocked or free is decided exactly, by the clearance: the distance from the origin to the convex hull of the face normals, not by sampling directions. Duck: at every valid R the four radial pairs block and all eight pieces are immobilized, yet there is no single key, because each blocked pair slides out together as a unit. Sphere: at R = 0 the same four radial pairs block, but only on a knife edge (quarter-sphere faces, exactly 90 degrees of spread). At R = 0.025 and 0.05 no pair blocks and all four outer shells (1, 3, 5, 7) come out alone, but with less than 2 degrees of clearance, piece 7 only 0.2 degrees. From R = 0.075 pair 6-7 is genuinely blocked, so shells 1, 3 and 5 come out and 5 of 8 are immobilized. R = 0.15 on the sphere and 0.02 on the duck fold. Four outer shells free means four candidate keys: not a single key.",
        S3)

# ---------------------------------------------------------------- matrix: who blocks whom
def matrix(r):
    blocked = {(p["a"], p["b"]) for p in r["pairs"] if p["blocked"]} | {(p["b"], p["a"]) for p in r["pairs"] if p["blocked"]}
    touch = {(p["a"], p["b"]) for p in r["pairs"]} | {(p["b"], p["a"]) for p in r["pairs"]}
    free = [1 if c >= 0 else 0 for c in r["pclr"]]
    cells = ['<p style="font-size:24px; color:#FFFFFF; text-align:center"></p>']
    for j in range(8):
        cells.append(f'<p style="font-size:24px; font-weight:700; text-align:center; color:{INK}">{j}</p>')
    for i in range(8):
        cells.append(f'<p style="font-size:24px; font-weight:700; text-align:center; color:{INK}">{i}</p>')
        for j in range(8):
            if i == j:
                bg, tx, col = "#FFFFFF", "", INK
            elif (i, j) in blocked:
                bg, tx, col = P2, "B", "#FFFFFF"
            elif (i, j) in touch:
                bg, tx, col = "#DCEBF5", "·", INK
            else:
                bg, tx, col = "#F4F5F6", "", INK
            cells.append(f'<p style="font-size:24px; font-weight:700; text-align:center; color:{col}; background:{bg}; padding:6px 0px; border-radius:4px">{tx}</p>')
    grid = (f'<div style="display:grid; grid-template-columns:repeat(9, 64px); gap:6px">{"".join(cells)}</div>')
    imm = [str(k) for k in range(8) if free[k] == 0]
    mov = [str(k) for k in range(8) if free[k] > 0]
    note = ("Immobilized: " + (", ".join(imm) if imm else "none") + ". Free alone: " + (", ".join(mov) if mov else "none") + ".")
    return grid, note
MX = {'sphere': {'shape': 'sphere', 'file': 'tvs_sphere_r010', 'R': 0.1, 'detJ': 0.1676, 'pairs': [{'a': 0, 'b': 4, 'axis': 'u', 'spread': 0.0, 'free': 2001, 'blocked': False, 'clr': 90.0}, {'a': 1, 'b': 5, 'axis': 'u', 'spread': 0.0, 'free': 2001, 'blocked': False, 'clr': 90.0}, {'a': 2, 'b': 6, 'axis': 'u', 'spread': 0.0, 'free': 2001, 'blocked': False, 'clr': 90.0}, {'a': 3, 'b': 7, 'axis': 'u', 'spread': 0.0, 'free': 2001, 'blocked': False, 'clr': 90.0}, {'a': 0, 'b': 2, 'axis': 'v', 'spread': 21.7, 'free': 1477, 'blocked': False, 'clr': 72.359}, {'a': 4, 'b': 6, 'axis': 'v', 'spread': 16.9, 'free': 1585, 'blocked': False, 'clr': 76.35}, {'a': 1, 'b': 3, 'axis': 'v', 'spread': 16.8, 'free': 1600, 'blocked': False, 'clr': 74.325}, {'a': 5, 'b': 7, 'axis': 'v', 'spread': 14.5, 'free': 1677, 'blocked': False, 'clr': 78.709}, {'a': 0, 'b': 1, 'axis': 'w', 'spread': 88.0, 'free': 34, 'blocked': False, 'clr': 2.947}, {'a': 2, 'b': 3, 'axis': 'w', 'spread': 97.3, 'free': 3, 'blocked': False, 'clr': 1.322}, {'a': 4, 'b': 5, 'axis': 'w', 'spread': 88.7, 'free': 18, 'blocked': False, 'clr': 2.76}, {'a': 6, 'b': 7, 'axis': 'w', 'spread': 99.6, 'free': 0, 'blocked': True, 'clr': 0.0}], 'free': [0, 1, 0, 1, 0, 1, 0, 0], 'pclr': [-1.0, 2.947, -1.0, 1.165, -1.0, 2.761, -1.0, -1.0], 'key': False, 'k': 0, 'ways': 9, 'smallest': '1', 'thin': 1.8865, 'ratio': 3.0672336978533816, 'folded': 0}, 'duck': {'shape': 'duck', 'file': 'tvs_duck_r000', 'R': 0, 'detJ': 0.03562, 'pairs': [{'a': 0, 'b': 4, 'axis': 'u', 'spread': 0.0, 'free': 2001, 'blocked': False, 'clr': 90.0}, {'a': 1, 'b': 5, 'axis': 'u', 'spread': 0.0, 'free': 2001, 'blocked': False, 'clr': 90.0}, {'a': 2, 'b': 6, 'axis': 'u', 'spread': 0.0, 'free': 2001, 'blocked': False, 'clr': 90.0}, {'a': 3, 'b': 7, 'axis': 'u', 'spread': 0.0, 'free': 2001, 'blocked': False, 'clr': 90.0}, {'a': 0, 'b': 2, 'axis': 'v', 'spread': 1.0, 'free': 1977, 'blocked': False, 'clr': 89.266}, {'a': 4, 'b': 6, 'axis': 'v', 'spread': 1.0, 'free': 1977, 'blocked': False, 'clr': 89.266}, {'a': 1, 'b': 3, 'axis': 'v', 'spread': 1.1, 'free': 1969, 'blocked': False, 'clr': 89.111}, {'a': 5, 'b': 7, 'axis': 'v', 'spread': 1.1, 'free': 1974, 'blocked': False, 'clr': 89.111}, {'a': 0, 'b': 1, 'axis': 'w', 'spread': 109.3, 'free': 0, 'blocked': True, 'clr': 0.0}, {'a': 2, 'b': 3, 'axis': 'w', 'spread': 118.9, 'free': 0, 'blocked': True, 'clr': 0.0}, {'a': 4, 'b': 5, 'axis': 'w', 'spread': 109.3, 'free': 0, 'blocked': True, 'clr': 0.0}, {'a': 6, 'b': 7, 'axis': 'w', 'spread': 118.9, 'free': 0, 'blocked': True, 'clr': 0.0}], 'free': [0, 0, 0, 0, 0, 0, 0, 0], 'pclr': [-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0], 'key': False, 'k': 0, 'ways': 6, 'smallest': '0,1', 'thin': 0.17278652906029332, 'ratio': 11.38211382113821, 'folded': 0}}
def mblock(name, r):
    g, note = matrix(r)
    return (f'<div style="flex:1; display:flex; flex-direction:column; gap:16px; align-items:center">'
            f'<h3 style="font-size:32px; font-weight:700; color:{INK}">{name}</h3>{g}'
            f'<p style="font-size:26px; color:{BODY}; text-align:center">{note}</p></div>')
content("matrix", "Step 4 · The blocking graph", "Who blocks whom",
        row(mblock("Sphere, R = 0.1", MX["sphere"]), mblock("Duck, R = 0", MX["duck"]), gap=64)
        + p("<b>B</b> = blocked: no direction separates the two pieces. <b>·</b> = the pieces touch but can slide apart. Empty = the pieces do not touch. Piece number = 4·(u half) + 2·(v half) + (w layer).", 24),
        "This is the directional blocking graph reduced to its adjacency form. Each piece touches three others: one across the u cut, one across the v cut, one across the radial w cut. On the duck the radial neighbour always blocks, so each row has exactly one B. That is why every duck piece is immobilized, and also why none of this interlocks: the only blocker of each piece is its radial partner, so the pair moves out together through the flat u and v cuts. On the sphere at R = 0.1 only pieces 6 and 7 block each other; the outer shells 1, 3 and 5 can leave alone.",
        S3)

# ---------------------------------------------------------------- 18 cuts
content("cuts", "Mechanism", "Only the radial cut is curved",
        row(col(table(["cut", "what it is", "sphere", "duck", "blocks?"], [14, 38, 14, 16, 18],
                      [["u", "a plane through the axis", "0°", "0°", "never"],
                       ["v", "a cone-like band, nearly flat", "0 to 11°", "1°", "never"],
                       ["w", "a scaled copy of the surface", "90°", "109 to 119°", "duck always"]], 25, 979),
                p("Spread of the normals on each cut. Distance from the best-fit plane on the sphere: u 0.4%, v 2.5%, w 41.5%.", 26),
                p("At R = 0 a u cut contains the axis (sphere) or is the mirror plane (duck), so it is a plane. R bends it only a little before the map folds, and on the sphere not at all: next slide.", 26, INK), flex="3"),
            shot("[Screenshot: GuIrit, cuts_u_flat, cuts_v_nearly, cuts_w_curved]", flex="2")),
        "Two independent measurements agree: the normal spread from the map, and a plane fitted to each cut surface's control points. The files cuts_u_flat.itd, cuts_v_nearly.itd and cuts_w_curved.itd in results show them. So eight of the twelve interfaces in this puzzle are the same flat cuts the original criticism was about. The reframing has only been exercised in one direction of three.",
        S3)

# ---------------------------------------------------------------- guard
content("guard", "Mechanism", "Why the sphere's u slice never bends",
        row(col(formula("RandomizeInterior:<br>if ( y != 0 &amp;&amp; ... ,  y = y + random( −R, R ) )", 28),
                bullets([
                    "Elber's function skips any coordinate equal to 0. It was written for box trivariates, whose coordinates are grid indices.",
                    "On the sphere the control rings at u = 0.25, 0.5 and 0.75 lie exactly in x = 0 or y = 0, and those knots are triple: each ring is the u slice.",
                    "So R never moves the slice out of its plane. At R = 0.2 the u = 0.5 ring moves up to 0.18 in x and z, and exactly 0 in y.",
                ], 26), flex="3"),
            col(table(["R", "u slice, guard kept", "u slice, removed", "single key"], [16, 30, 30, 24],
                      [["0.025", "0°", "5 to 6°", "no"], ["0.05", "0°", "11 to 14°", "no"], ["0.1", "0°", "22 to 32°", "no"], ["0.125", "0°", "28 to 57°", "no"], ["0.15", "folds", "folds", "-"]], 24, 653),
                p("Same random offsets in both columns. Removing the guard curves u, but never past about 90°, so it still blocks nothing. The duck is unaffected.", 26, INK), flex="2")),
        "This is why PuzzleVolEarth shows curved v and w slices but a perfectly flat u slice. It was checked by comparing control points of the saved trivariates, then isolated with RandomizeInteriorAll: the same random draws applied with the guard kept (tvsg files) and removed (tvsa files). The two differ in 36 of 416 control points, all coordinates that were zero. Result: the u cut bends to at most 57 degrees before the map folds at R = 0.15, and the blocking pattern is unchanged; still no single key. The duck has no control point with a zero coordinate, so the guard never touched it. Conclusion: the flat cuts are not only a quirk of the function; a random perturbation simply cannot bend them past the 90 degrees blocking needs before det J goes negative. Curvature has to be designed in.",
        S3)

# ---------------------------------------------------------------- 19 pairs
content("pairs", "Why there is no key · duck", "Each piece has exactly one blocker",
        row(col(card("Pairs escape", "Every piece touches three others: across u (flat), across v (nearly flat), across w (blocked). Pair it with its w-partner and the only blocking face becomes internal. The pair leaves through the flat cuts.", flex="none", accent=ORANGE, size=26),
                card("Pairs never separate", "Each radial pair is blocked in every direction, so the take-apart always ends with a pair stuck together. By translation the duck could not be built from 8 loose pieces either.", flex="none", accent=INK, size=26), flex="3", gap=20),
            shot("[Screenshot: Analyse puzzle window, duck R = 0, pair {0, 1} pulled out]", flex="2")),
        "This is why the numbers are constant across every valid duck row: 0 pieces free alone, 6 ways to open, smallest group 2. It is the structure of the division, not noise. An earlier write-up called the duck over-locked; that was wrong. The puzzle as a whole comes apart, a pair at a time; it is each pair that is locked. Elber warns in section 6 that the construction can produce puzzles that cannot be disassembled; this is a measured instance.",
        S3)

# ---------------------------------------------------------------- 20 knife
content("knife", "Why there is no key · sphere", "The sphere sits exactly on the threshold",
        p("Each radial face is a quarter of a sphere: its normals spread exactly 90°. That is the boundary between free and blocked. The outer shell can leave along one exact line of directions, and nowhere else.", 32, INK)
        + row(card("R = 0", "No sampled direction lands exactly on that line, so the DBG reports 4 blocked pairs. Sliding the shell out along it works: the one disagreement in the slide check.", accent=BLUE),
              card("R &gt; 0", "The perturbation tips each pair to one side. R = 0.025 and 0.05: all four open, every outer shell comes out, with under 2° of clearance. From R = 0.075: pair 6–7 closes.", accent=ORANGE))
        + p("Blocking that flips with a 2% perturbation is not a lock.", 32, INK),
        "Geometrically, the set of free directions for the outer shell at R = 0 is a one-dimensional arc on the sphere of directions: exactly zero clearance, blocked in the exact test too. With R above zero each pair tips to one side by a fraction of a degree: at R = 0.05 the clearances are 1.65, 1.00, 1.30 and 0.19 degrees for shells 1, 3, 5 and 7. A 4000-direction sample missed shell 7 completely, which is why the analysis now uses the exact clearance. The duck's radial faces, at 109 to 119 degrees of spread, are clearly past the threshold: clearance 0 at every R.",
        S3)

# ---------------------------------------------------------------- 21 fold
content("fold", "Past the fold", "Bigger R looks more locked, and isn't",
        row(col(table(["file", "folded pieces", "blocked", "ways to open"], [34, 22, 22, 22],
                      [["duck R = 0.03", "8 of 8", "4 of 12", "3"],
                       ["duck R = 0.05", "8 of 8", "7 of 12", "1"],
                       ["duck R = 0.1", "8 of 8", "12 of 12", "0"],
                       ["sphere R = 0.5", "8 of 8", "9 of 12", "0"]], 26, 979),
                p("Duck R = 0.1 reads as the best row in the study: nothing can move. Every one of its pieces passes through itself. Blocking from a folded map is not interlocking.", 28, INK), flex="3"),
            shot("[Screenshot: Analyse puzzle window, duck R = 0.1, folded pieces in red]", flex="2")),
        "These larger R values were generated on purpose, to see the failure. Without the Jacobian column, R = 0.1 on the duck would look like the fully interlocked result we want. It is the worst case: the rubber cube has turned inside out over much of the domain. This is the practical reason the positive-Jacobian criterion is on the list.",
        S3)

# ---------------------------------------------------------------- 22 division
content("division", "Does refining help?", "More layers make a bigger escaping unit",
        table(["division", "duck R = 0: blocked / ways / smallest", "sphere R = 0.05: blocked / ways / smallest"], [20, 40, 40],
              [["2×2×1", "0 / 6 / 1", "0 / 6 / 1"],
               ["2×2×2", "4 / 6 / 2", "0 / 10 / 1"],
               ["2×2×4", "12 / 6 / 4", "3 / 14 / 1"],
               ["3×3×2", "0 / 181 / 1", "0 / 290 / 1"]], 28)
        + p("Layers through the thickness make the duck's escaping unit a whole radial column of 4. More pieces around the surface unblock everything. While u and v are flat, the ceiling is 'a column escapes'.", 28, INK)
        + p("Every group tested in every row. Up to 12 pieces decided exactly; the 16 and 18-piece rows by sampled directions.", 26),
        "At 2x2x4 every radial interface on the duck blocks and no other one does; the only ways out are radial columns. At 3x3x2 each radial face covers a third of the turn instead of half, so its normals spread less than 90 degrees and nothing blocks. Refining the division cannot produce a single key while two of the three cut families are flat.",
        S3)

# ---------------------------------------------------------------- 23 scorecard: criteria per R
CRIT_HEAD = ["R", "det J &gt; 0", "thinnest (limit 3%)", "largest / smallest", "key / level k"]
CRIT_W = [12, 18, 28, 24, 18]
content("scorecard", "Criteria", "The criteria, per R",
        row(shape_block("Sphere", table(CRIT_HEAD, CRIT_W, [['0', 'yes', '8.1%, pass', '2.7, warn', 'no / 0'], ['0.025', 'yes', '6.0%, pass', '2.8, warn', 'no / 0'], ['0.05', 'yes', '4.2%, pass', '2.9, warn', 'no / 0'], ['0.075', 'yes', '2.7%, fail', '3.0, warn', 'no / 0'], ['0.1', 'yes', '1.9%, fail', '3.1, fail', 'no / 0'], ['0.125', 'yes', '0.5%, fail', '3.2, fail', 'no / 0'], ['0.15', 'no, folds', '-', '-', '-']], 24, 816)),
            shape_block("Duck", table(CRIT_HEAD, CRIT_W, [['0', 'yes', '0.2%, fail', '11.4, fail', 'no / 0'], ['0.0025', 'yes', '0.2%, fail', '11.4, fail', 'no / 0'], ['0.005', 'yes', '0.2%, fail', '11.5, fail', 'no / 0'], ['0.0075', 'yes', '0.2%, fail', '11.5, fail', 'no / 0'], ['0.01', 'yes', '0.2%, fail', '11.5, fail', 'no / 0'], ['0.015', 'yes', '0.2%, fail', '11.7, fail', 'no / 0'], ['0.02', 'no, folds', '-', '-', '-']], 24, 816)))
        + p("<b>One connected component</b>: yes in every valid row. A piece is one box pushed through a continuous map, so only a fold can break it. <b>Joints in cavities</b>: not applicable, this experiment has no joints. Sizes pass at 2 or less, warn up to 3.", 24),
        "Every criterion from the problem statement, per R. Positive Jacobian holds up to R = 0.125 on the sphere and R = 0.015 on the duck. Thickness: the sphere passes the 3% limit up to R = 0.05 and falls below it after; the duck is thin at the neck, 0.2% of the model, at every R. Sizes: the sphere's inner pieces are smaller than its outer shells, ratio about 3; the duck's head pieces are about 11 times smaller than its body pieces. The 3% thickness limit is my default, not a printing requirement. No valid row has a single key, so level k is 0 everywhere.",
        S3)

# ---------------------------------------------------------------- 24 elber
content("elber", "Elber 2025, §6 Future Work", "Each §6 item, matched to our plan",
        table(['Elber §6 says', 'our plan', 'status now'], [36, 33, 31],
              [['1 · Other domain topologies: hexagonal prisms, semi-regular or general space-filling cells, with matching joints', "Not in our plan. We keep Elber's cuboid grid; we change whether the cells interlock, not their shape", '–'], ['2 · Non-regular division, plus automatic verification of a collision-free (dis)assembly ("herein we manually ensure that")', 'Our core step: the DBG tests every group of pieces for a free translation and builds a take-apart sequence', 'Done for uniform 2×2×2 (complete up to 18 pieces), translation only. Not yet: non-regular division, rotations'], ['3 · Better joints than pins and holes; deep cavities can expose joints', 'Later, only if curvature alone does not lock: the DBG shows which faces are loose, only those get a joint', 'Not started: no joints in this experiment; dovetails exist only in the older planar pipeline; no cavity check'], ["4 · Pin and hole sizes distorted by M; compensate with M's derivatives", "Later, together with joints: the analyser already evaluates M's derivatives (J) at any point", 'Not started: no compensation implemented']], 24),
        "Item 1 is not part of our plan: we keep the cuboid grid. Item 2 is where this work sits: Elber ensures (dis)assemblability manually and names automatic verification as open; the DBG automates it, so far for uniform grids and translations only. The duck's radial pairs are the case he warns about, a puzzle that cannot be (dis)assembled. Items 3 and 4 concern joints, which this experiment deliberately removed; they become relevant only if curvature alone does not lock. Source: Elber, Modelling 2025, 6, 90, section 6.",
        S4)

# ---------------------------------------------------------------- 25 oq1
content("oq1", "Open questions", "Scope: which motions, which shapes",
        row(card("1 · Translation only, or rotations too?", "The d · n test is about straight pulls. A curved face that blocks every translation may still allow a screw motion, and rotations turn the cone test into a 6-D search. Proposal: translation only for the first paper, stated as a limitation, plus a swept-volume check on the final design.", accent=ORANGE),
            card("2 · Which shapes are in scope?", "Proposal: genus 0 and no limbs, for example a deformed sphere or a simple bust. That excludes the torus and the armadillo, whose cross-sections split into several loops. Question: is 'no limbs' about genus, the medial axis, or convex cross-sections? It needs an operational test.", accent=BLUE)),
        "Earlier in the project a cuboid piece could not rotate in place in a snug packing at any angle, which supported ignoring rotations for planar cells; that result does not transfer to curved interfaces. On shapes: I would start with genus 0 and no limbs. The armadillo was measured earlier to produce multi-loop cross-sections and disconnected lumps when cut, which is the practical reason to exclude limbs.",
        S4)

# ---------------------------------------------------------------- 26 oq2
def smallcard(head, body, accent):
    return (f'<div style="display:flex; flex-direction:column; gap:12px; background:{CARD}; padding:32px; border-top:6px solid {ACCENTS.get(accent, accent)}; border-radius:8px">'
            f'<h3 style="font-size:32px; font-weight:700; line-height:1.25; color:{INK}">{head}</h3>'
            f'<p style="font-size:26px; line-height:1.4; color:{BODY}">{body}</p></div>')
content("oq2", "Open questions", "Method: definitions and instruments",
        f'<div style="display:grid; grid-template-columns:1fr 1fr; gap:24px">'
        + smallcard("3 · Is random R the right instrument?", "R folds the duck at 0.02 and never changes which cuts block. Shape the interior deliberately instead: a harmonic trivariate (Martin, Cohen &amp; Kirby 2009)?", ORANGE)
        + smallcard("4 · Which level k?", "Song 2012 (a unique key at each step) is computed. Chen 2022 (moves before the first piece comes free) needs motion search. Which is the claim?", BLUE)
        + smallcard("5 · What do joints become?", "If curvature does not lock, the DBG says which faces are loose. Is the objective the fewest joints, or the least visible ones?", INK)
        + smallcard("6 · Fitted or constructed trivariates?", "So far the V-reps are constructed. Claim results on fitted meshes, or name the fit as future work? And what thickness limit, in mm?", INK)
        + '</div>',
        "Question 3 is really the choice between tuning a random perturbation and designing the trivariate. The data say R is the wrong knob: it has no effect on which cuts block, and it folds the duck almost immediately. Question 6 matters for the paper's claim: the current trivariates are Elber's constructed ones, not fitted to scanned meshes.",
        S4)

# ---------------------------------------------------------------- 27 next
content("next", "Next step", "Make the other two cuts curve",
        row(card("A · Wavy cuts in D (days)", "Replace the flat u and v cut planes in D with gently curved surfaces. Same trivariate, same analyser. It tests the hypothesis directly: if it gives a single key, that is the first positive result.", accent=ORANGE),
            card("B · Harmonic trivariate (weeks)", "Build M from a harmonic field so all three directions curve and the Jacobian has headroom. The principled version, as suggested last meeting.", accent=BLUE))
        + p("Recommendation: A first. Its result decides whether B is worth the effort.", 32, INK),
        "Both address the same blocker: on a surface-of-revolution trivariate, u and v cuts are flat or nearly flat, so only the radial cut can block. Option A breaks that in the domain without a new trivariate; Elber's section 6 already raises non-regular divisions of D. Option B fixes it at the source and also gives Jacobian headroom, which the duck lacks. Everything built so far, the DBG, the slide check and the criteria, applies unchanged to either.",
        S5)

# ---------------------------------------------------------------- 28-29 Q&A
qa1 = [["Why not analyse the IRIT piece meshes?", "Meshes give facet normals only and have no Jacobian. We divide the same M; piece boxes match the tiles to 0.01."],
       ["Is M_u × M_v really J⁻ᵀn?", "Yes. The rows of J⁻ᵀ are the cofactors over det J, so J⁻ᵀe_w = (M_u × M_v) / det J."],
       ["How do you know the sign is right?", "The slide check: the group clears along +d and collides along −d on every shape."],
       ["Are 4000 sampled directions enough?", "No: they missed a 0.19° sliver. Blocked or free is decided exactly, by the clearance."],
       ["Is 'no piece is free' interlocking?", "No. Groups decide: the duck has 0 free pieces, but its pairs escape."],
       ["Why does the duck come apart 2 pieces at a time?", "Each piece's only blocker is its radial partner. Together, that face is inside the pair; only flat cuts remain."]]
qa2 = [["Why does R not help?", "Two cut families start flat; R bends them at most 45 to 57° before the map folds, short of the ~90° blocking needs."],
       ["Why is PuzzleVolEarth's u slice flat when v and w bend?", "Elber's RandomizeInterior never moves a coordinate equal to 0, and those control rings lie in x = 0 or y = 0."],
       ["Elber never cuts through the thickness. Why do we?", "The professor asked for 2×2×2. Without the radial cut nothing blocks (2×2×1: 0 of 4)."],
       ["How is thickness measured?", "Inward rays from face points; Newton inversion finds where each ray leaves the piece."],
       ["What about rotations?", "Not checked: assemblable under translational blocking; rotational/swept check pending."],
       ["Can I see it?", "Yes: the app's Analyse puzzle window. Open a tvs file, click an opening, drag Pull, press Verify."]]
content("qa1", "Backup", "Questions I expect: the method",
        table(["question", "answer"], [38, 62], qa1, 26),
        "Backup slide. The cofactor identity: for J with columns a = M_u, b = M_v, c = M_w, the inverse is (1/det J) times the matrix whose rows are b × c, c × a and a × b. So the transformed normal of a w = const face is proportional to a × b. The knife-edge answer: the DBG can only fail by calling a pair blocked when its free set is thinner than the direction sampling; the slide check exists to catch exactly that, and it did on the sphere at R = 0.",
        S5)
content("qa2", "Backup", "Questions I expect: the results",
        table(["question", "answer"], [38, 62], qa2, 26),
        "Backup slide. Everything is reproducible: python make_tables.py in meeting_feedback regenerates the blocking table and ranking from the saved trivariates; the Analyse puzzle window in the app shows each case with its pieces, openings, take-apart and criteria, and runs the slide check on demand.",
        S5)

# ---------------------------------------------------------------- tool slide (inserted after verify)
content("tool", "Tooling", "Everything above, in one window",
        row(col(bullets([
            "Open any tvs_*.itd, choose the division, press <b>Analyse</b>: about one second for 2×2×2.",
            "Criteria as PASS, FAIL, WARN or N/A, with the numbers behind each.",
            "<b>Ways it opens</b>: click one and drag <b>Pull</b> to slide it out along its own direction.",
            "<b>Verify by sliding</b> runs the independent check on the selected group.",
            "<b>Disassembly</b> plays the take-apart. Orange: moving. Red: folded. Yellow: too thin.",
        ]), p("Tables for the whole sweep: python make_tables.py", 26, INK), flex="3"),
            shot("[Screenshot: Analyse puzzle window]", flex="2")),
        "The analyser is part of the Qt app: a button Analyse puzzle in the main window opens it. It uses the same code as the command line and the table generator, so what is on screen is exactly what is in the tables.",
        S2)

order = ["cover", "reframe", "dmap", "statement", "setup", "rparam",
         "dbgflow", "test", "normals", "directions", "groups", "keylevel", "verify", "tool", "jacobian", "shape",
         "headline", "results", "matrix", "cuts", "guard", "pairs", "knife", "fold", "division", "scorecard",
         "elber", "oq1", "oq2", "next", "qa1", "qa2"]
ids = {s for s, _ in slides}
assert set(order) == ids, (set(order) ^ ids)
for sid, html in slides:
    io.open(os.path.join(SL, sid + ".html"), "w", encoding="utf-8").write(html.replace("{NUM}", str(order.index(sid) + 1)))

deck = {
    "v": 4,
    "createdOnFiles": {"v": 1, "at": "2026-09-26T09:55:00Z"},   # fixed at creation; keep
    "title": "Interlocking by Curvature",
    "order": order,
    "sections": {
        "s1": {"description": "The question, the reframing and the experimental setup", "start": "cover"},
        "s2": {"description": "How the DBG, the Jacobian and the shape criteria are computed and checked", "start": "dbgflow"},
        "s3": {"description": "The results: blocking table, mechanism, folding, division, scorecard", "start": "headline"},
        "s4": {"description": "Elber 2025 section 6 mapped to our plan, and the open questions", "start": "elber"},
        "s5": {"description": "The next step and backup answers to likely questions", "start": "next"},
    },
    "faces": {
        "noto-sans-kr": {"family": "Noto Sans KR", "href": "https://fonts.googleapis.com/css2?family=Noto+Sans+KR:wght@400;500;700&display=swap"},
        "ibm-plex-mono": {"family": "IBM Plex Mono", "href": "https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400&display=swap"},
    },
    "designSystems": [],
}
io.open(os.path.join(ROOT, "project", "deck.json"), "w", encoding="utf-8").write(json.dumps(deck, indent=1, ensure_ascii=False))
print(len(slides), "slides")
for sid, html in slides:
    a = html.index("<aside>")
    n = len(html[a:])
    if n > 4000: print("NOTES TOO LONG", sid, n)
