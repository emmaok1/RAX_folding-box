import sys
from pathlib import Path
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE, MSO_CONNECTOR
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.oxml.ns import qn
from lxml import etree

OUT = Path(sys.argv[1])
IMG = OUT / 'images'

NAVY = RGBColor(0x1F, 0x2A, 0x44)
ORANGE = RGBColor(0xF2, 0x6B, 0x21)
GREY = RGBColor(0x6B, 0x72, 0x80)
LIGHT = RGBColor(0xF3, 0xF4, 0xF6)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
RED = RGBColor(0xC0, 0x39, 0x2B)
GREEN = RGBColor(0x2E, 0x7D, 0x32)
LAYER = [RGBColor(0xE8, 0xEE, 0xF8), RGBColor(0xFD, 0xEB, 0xDF), RGBColor(0xE6, 0xF4, 0xEA),
         RGBColor(0xFF, 0xF4, 0xD6), RGBColor(0xEE, 0xE8, 0xF6), RGBColor(0xE3, 0xF2, 0xF7)]

prs = Presentation()
prs.slide_width, prs.slide_height = Inches(13.333), Inches(7.5)
BLANK = prs.slide_layouts[6]


def text(slide, x, y, w, h, runs, size=12, color=NAVY, bold=False, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame; tf.word_wrap = True; tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Inches(0.04); tf.margin_top = tf.margin_bottom = Inches(0.02)
    fill_tf(tf, runs, size, color, bold, align)
    return tb


def fill_tf(tf, runs, size, color, bold, align):
    """runs: str | list of paragraphs; paragraph = str | list of (text, {opts})"""
    if isinstance(runs, str):
        runs = [runs]
    for i, para in enumerate(runs):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align
        if isinstance(para, str):
            para = [(para, {})]
        for t, o in para:
            r = p.add_run(); r.text = t
            r.font.size = Pt(o.get('size', size)); r.font.bold = o.get('bold', bold)
            r.font.color.rgb = o.get('color', color); r.font.name = 'Calibri'
            r.font.italic = o.get('italic', False)


def box(slide, x, y, w, h, fill, runs=None, size=11, color=NAVY, bold=False, line=None, dash=False,
        shape=MSO_SHAPE.ROUNDED_RECTANGLE, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.MIDDLE):
    s = slide.shapes.add_shape(shape, Inches(x), Inches(y), Inches(w), Inches(h))
    if shape == MSO_SHAPE.ROUNDED_RECTANGLE:
        s.adjustments[0] = 0.08
    s.fill.solid(); s.fill.fore_color.rgb = fill
    if line is None:
        s.line.fill.background()
    else:
        s.line.color.rgb = line; s.line.width = Pt(1.25)
        if dash:
            s.line.dash_style = 4  # dash
    s.shadow.inherit = False
    tf = s.text_frame; tf.word_wrap = True; tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Inches(0.08); tf.margin_top = tf.margin_bottom = Inches(0.04)
    if runs is not None:
        fill_tf(tf, runs, size, color, bold, align)
    return s


def arrow(slide, x1, y1, x2, y2, color=GREY, width=1.75, dash=False):
    c = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(x1), Inches(y1), Inches(x2), Inches(y2))
    c.line.color.rgb = color; c.line.width = Pt(width)
    if dash:
        c.line.dash_style = 4
    ln = c.line._get_or_add_ln()
    tail = etree.SubElement(ln, qn('a:tailEnd')); tail.set('type', 'triangle'); tail.set('w', 'med'); tail.set('len', 'med')
    return c


def header(slide, n, title, subtitle):
    box(slide, 0, 0, 13.333, 0.95, NAVY, shape=MSO_SHAPE.RECTANGLE)
    text(slide, 0.4, 0.1, 10.5, 0.5, title, size=26, color=WHITE, bold=True)
    text(slide, 0.4, 0.55, 11.5, 0.35, subtitle, size=12.5, color=RGBColor(0xC9, 0xD3, 0xE6))
    box(slide, 12.45, 0.25, 0.5, 0.45, ORANGE, str(n), size=16, color=WHITE, bold=True, align=PP_ALIGN.CENTER)


def footer(slide, t):
    text(slide, 0.4, 7.17, 12.5, 0.3, t, size=9, color=GREY)


def picture(slide, path, x, y, w=None, h=None, caption=None, link=None):
    pic = slide.shapes.add_picture(str(path), Inches(x), Inches(y), Inches(w) if w else None, Inches(h) if h else None)
    pic.line.color.rgb = RGBColor(0xD1, 0xD5, 0xDB); pic.line.width = Pt(0.75)
    if caption:
        ph = pic.height / 914400
        text(slide, x, y + ph + 0.02, (pic.width / 914400), 0.3, caption, size=10, color=GREY)
    return pic


# ---------------------------------------------------------------- slide 1
s = prs.slides.add_slide(BLANK)
header(s, 1, 'Current Architecture',
       'Dexmate Vega 1U box-folding baseline  ·  repo commit 54a9a8b + local experiments  ·  as of 2026-11-04')

layers = [
    ('Agent / Cloud', 'NOT CONNECTED YET',
     [[('RAX', {'bold': True}), (' (Robot Agent eXploration, upstream dexjoco/native @4b577aa, unmodified)  ·  a coding agent '
       '(Claude Code or Codex CLI) explores through MCP tools, writes controller.py, keeps memory '
       '(PROGRESS / TIPS / verified skills / handoff) in its workspace', {})],
      [('Connecting RAX ↔ box simulation is future work; no LLM / cloud API is in the current loop.', {'color': RED, 'italic': True})]]),
    ('Experiment drivers', 'validation/*.py',
     ['Scripted, deterministic tests read MuJoCo ground-truth state (panel poses, contacts) and choose targets: '
      'feasibility_tester · edge_exposure · side_push · slow_contact · robot_contact · community_*']),
    ('Motion planning', 'OMPL 2.0.1',
     [[('OMPL RRTConnect', {'bold': True}), (' in 7-DoF right-arm joint space (fixed seed, 0.005 rad validation step)  ·  IK: '
       'FoldingBoxEnv.inverse_kinematics (damped least squares)  ·  every path re-checked densely on the original MuJoCo '
       'collision model  ·  compared: Tesseract TrajOpt + OSQP (approach timed out → OMPL kept)', {})]]),
    ('Control', 'environment.py',
     [[('Joint PD + gravity compensation', {'bold': True}), (' (kp 180→40 N·m/rad, kd 22→6), 50 Hz via '
       'FoldingBoxEnv.step() (Cartesian → IK) or step_joint_targets()  ·  jaws: PD, 0.4 N·m closing torque  ·  '
       'experimental: vega_curobo GeometricImpedanceController on right arm (experiments/vega_impedance, CPU only)', {})]]),
    ('Simulation', 'MuJoCo 3.4.0',
     ['implicitfast integrator, 0.25 ms physics step, 80 substeps per 20 ms control row  ·  models/scene.xml '
      '(robot + box + table inline)  ·  contacts/forces monitored every substep in experiments']),
    ('Robot & object', 'scene.xml',
     [[('Dexmate Vega 1U', {'bold': True}), (' (wrist-FT + DexGripper S): 2 × 7-DoF arms, coupled parallel jaws with custom sloped pads, '
       '4 RGB cameras (back, head, 2 wrist)  ·  ', {}), ('Box', {'bold': True}),
       (': 14 rigid 4 mm panels, 13 passive creases, 300×220 mm base, 90 mm walls, 107 g', {})]]),
]
y0, lh, gap = 1.15, 0.8, 0.13
for i, (name, tag, body) in enumerate(layers):
    y = y0 + i * (lh + gap)
    dashed = i == 0
    box(s, 0.4, y, 2.15, lh, NAVY if not dashed else RGBColor(0x9C, 0xA3, 0xAF),
        [[(name, {'bold': True, 'size': 13, 'color': WHITE})], [(tag, {'size': 9.5, 'color': RGBColor(0xE5, 0xE7, 0xEB)})]],
        align=PP_ALIGN.CENTER)
    box(s, 2.65, y, 9.45, lh, LAYER[i] if not dashed else WHITE, body, size=11.5,
        line=RGBColor(0x9C, 0xA3, 0xAF) if dashed else None, dash=dashed)
    if i < len(layers) - 1:
        arrow(s, 7.35, y + lh, 7.35, y + lh + gap, dash=(i == 0))
# feedback arrow on right
fb_top, fb_bot = y0 + (lh + gap) + 0.2, y0 + 5 * (lh + gap) + lh - 0.2
arrow(s, 12.95, fb_bot, 12.95, fb_top, color=ORANGE, width=2.25)
text(s, 12.13, fb_top + 1.3, 0.8, 2.2, 'measured state, contacts & forces feed back', size=9.5, color=ORANGE, align=PP_ALIGN.CENTER)
footer(s, 'Earlier integrated prototype (Claude Sonnet 4.6 via bounded relay → RAX MCP "teacher" role → MotionService/OMPL 1.7) was archived '
          'at commit c6f5352 and is not part of the current checkout.')

# ---------------------------------------------------------------- slide 2
s = prs.slides.add_slide(BLANK)
header(s, 2, 'The MuJoCo Scene', 'Room, table, Dexmate Vega 1U and the flat cardboard blank  ·  rendered from the unchanged models/scene.xml')
picture(s, IMG / 'scene_full.png', 0.4, 1.15, w=6.4, caption='Whole scene: room, table, robot (home pose), flat blank')
gx, gy, gw = 7.0, 1.15, 2.95
cells = [('arm_right.png', 'Right 7-DoF arm reaching over the table'),
         ('gripper_right.png', 'DexGripper S claw: coupled parallel jaws, dark custom pads, wrist camera'),
         ('box_oblique.png', 'Flat blank between the two grippers'),
         ('box_top.png', 'Box blank, top view: base, 4 walls, 4 corner flaps, lid + flaps, tongue & slot')]
for k, (f, cap) in enumerate(cells):
    cx = gx + (k % 2) * (gw + 0.18); cy = gy + (k // 2) * 2.42
    picture(s, IMG / f, cx, cy, w=gw)
    text(s, cx, cy + gw * 1080 / 1620 + 0.02, gw, 0.45, cap, size=9.5, color=GREY)
# cameras strip
cy = 5.72
text(s, 0.4, cy - 0.02, 6.4, 0.3, [[('Robot cameras:', {'bold': True}), ('  back  ·  head / ego  ·  left wrist  ·  right wrist', {'color': GREY})]], size=10, color=NAVY)
for k, (f, cap) in enumerate([('cam_back.png', 'back'), ('cam_head.png', 'head / ego'),
                              ('cam_handcam_rgb_left.png', 'left wrist'), ('cam_handcam_rgb_right.png', 'right wrist')]):
    picture(s, IMG / f, 0.4 + k * 1.62, cy + 0.27, w=1.5)
# attachments
ax, ay = 7.0, 5.95
box(s, ax, ay, 6.1, 1.1, LIGHT, None, line=RGBColor(0xD1, 0xD5, 0xDB))
text(s, ax + 0.1, ay + 0.03, 5.9, 0.3, [[('Attached model files', {'bold': True}),
     ('  (in attachments/ next to this deck; click to open)', {'color': GREY, 'size': 9.5})]], size=11)
files = [('scene.xml', 'robot + box + table'), ('robot_config.json', 'robot params'),
         ('panels.json', 'box panels/creases'), ('physics.json', 'box material'), ('scene_bundle.zip', 'runnable, w/ meshes')]
for k, (f, d) in enumerate(files):
    fx = ax + 0.1 + k * 1.19
    b = box(s, fx, ay + 0.38, 1.12, 0.62, WHITE, [[(f, {'bold': True, 'size': 9, 'color': ORANGE})], [(d, {'size': 8, 'color': GREY})]],
            line=ORANGE, align=PP_ALIGN.CENTER)
    b.click_action.hyperlink.address = 'attachments/' + f
footer(s, 'STATIC ILLUSTRATIONS: reset state + mj_forward only, no physics stepping or control: not an experiment result. '
          'scene.xml needs models/ meshes; use scene_bundle.zip to load it.')

# ---------------------------------------------------------------- slide 3
s = prs.slides.add_slide(BLANK)
header(s, 3, 'Task & Process Pipeline',
       'Goal: fold the flat connected-lid blank into a closed, locked box with two Dexmate arms, through real contact only')
# steps row
text(s, 0.4, 1.08, 6, 0.3, 'Task stages (order chosen by the agent, not hard-coded)', size=12.5, bold=True)
steps = ['Stabilize the blank', 'Raise 4 body walls', 'Fold 4 corner flaps in', 'Stable open box',
         'Fold lid side & front flaps', 'Close the lid', 'Insert tongue in slot', 'Release, hold ≥ 5 s']
sw = 1.5
for k, t in enumerate(steps):
    x = 0.4 + k * (sw + 0.07)
    box(s, x, 1.42, sw, 0.62, LAYER[1] if k < 4 else LAYER[4],
        [[(f'{k + 1}  ', {'bold': True, 'color': ORANGE, 'size': 13}), (t, {'size': 10.5})]], align=PP_ALIGN.CENTER)
# success criteria
box(s, 0.4, 2.2, 4.3, 2.75, LIGHT, None)
text(s, 0.52, 2.25, 4.1, 0.3, 'Success benchmark  (environment.progress())', size=12, bold=True)
crit = ['Blank stabilized on table (box speed < 0.03 m/s)',
        'Walls & corners at 90° ± 12.6°, base upright',
        'Lid, lid-front & lid-side flaps at 90° ± 12.6°',
        'Tongue / slot geometric capture on every tab',
        'No robot contact; hinges < 0.05 rad/s',
        'Penetration < 0.1 mm',
        'Shape persists ≥ 5 s after release',
        'Physical locking independently verified']
text(s, 0.52, 2.58, 4.1, 2.6, [[('✓  ', {'color': GREEN, 'bold': True}), (c, {})] for c in crit[:-1]] +
     [[('✗  ', {'color': RED, 'bold': True}), (crit[-1] + ': not validated, so succeed is hard-coded False', {'color': RED})]],
     size=10.5)
# pipeline flow (current loop)
text(s, 4.95, 2.2, 8, 0.3, 'How one manipulation step runs today (scripted experiment loop)', size=12, bold=True)
flow = [('SENSE', 'MuJoCo ground truth: panel poses, crease angles, contacts; joints/TCP; 4 RGB cams'),
        ('DECIDE', 'Experiment script picks target TCP pose (planned: RAX agent)'),
        ('IK', 'DLS inverse kinematics → 7 joint goals'),
        ('PLAN', 'OMPL RRTConnect, collision-free, dense MuJoCo re-check'),
        ('EXECUTE', 'Joint refs ≤ 0.4 rad/s; final contact 0.5–5 mm/s'),
        ('CONTROL', 'PD + gravity torques @ 50 Hz'),
        ('PHYSICS', 'MuJoCo 0.25 ms steps; abort > 5 N or > 1 mm')]
fx0, fy, fw, fh, fg = 4.95, 2.6, 1.0, 1.6, 0.12
for k, (t, d) in enumerate(flow):
    x = fx0 + k * (fw + fg)
    box(s, x, fy, fw, fh, LAYER[k % 6], [[(t, {'bold': True, 'size': 10.5, 'color': ORANGE})], [(d, {'size': 9})]],
        align=PP_ALIGN.CENTER, anchor=MSO_ANCHOR.TOP)
    if k < len(flow) - 1:
        arrow(s, x + fw, fy + fh / 2, x + fw + fg, fy + fh / 2)
# feedback loop
lx_end = fx0 + 6 * (fw + fg) + fw / 2
c1 = arrow(s, lx_end, fy + fh, lx_end, fy + fh + 0.35, color=ORANGE)
ln = s.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(fx0 + fw / 2), Inches(fy + fh + 0.35), Inches(lx_end), Inches(fy + fh + 0.35))
ln.line.color.rgb = ORANGE; ln.line.width = Pt(1.75)
arrow(s, fx0 + fw / 2, fy + fh + 0.35, fx0 + fw / 2, fy + fh, color=ORANGE)
text(s, 6.3, fy + fh + 0.37, 5, 0.3, 'measured state & contact forces → next decision', size=9.5, color=ORANGE, align=PP_ALIGN.CENTER)
# two branches note
box(s, 4.95, 4.85, 3.8, 1.2, LAYER[2],
    [[('Free-space branch', {'bold': True})], 'Approach, retreat: IK → OMPL path → joint references → PD. Collision-checked.'],
    size=11, anchor=MSO_ANCHOR.TOP)
box(s, 8.87, 4.85, 3.8, 1.2, LAYER[1],
    [[('Contact branch', {'bold': True})], 'Touch, push, pinch, fold: small Cartesian steps via env.step (IK → PD). No contact planner or force controller.'],
    size=11, anchor=MSO_ANCHOR.TOP)
# planned
box(s, 0.4, 5.1, 4.3, 1.9, WHITE,
    [[('Planned agent loop (not yet connected)', {'bold': True, 'color': GREY})],
     [('Claude Code agent ⇄ cloud API for reasoning;  agent ⇄ RAX MCP tools for observe / plan / step;  '
       'RAX memory keeps notes & verified skills across episodes. It would replace the DECIDE box.', {'color': GREY})]],
    size=11, line=RGBColor(0x9C, 0xA3, 0xAF), dash=True, anchor=MSO_ANCHOR.TOP)
box(s, 4.95, 6.25, 7.72, 0.75, LAYER[0],
    [[('Status: ', {'bold': True}), ('free-space approach + gentle contact work reliably;  ', {}),
      ('no stable grasp, wall fold or locking demonstrated yet.', {'bold': True, 'color': RED})]], size=10.5)
footer(s, 'Benchmarks from environment.py progress(); 0.22 rad tolerance = 12.6°. Flow reflects validation/* experiments on commit 54a9a8b.')

# ---------------------------------------------------------------- slide 4
s = prs.slides.add_slide(BLANK)
header(s, 4, 'Open Problems', 'What blocks the fold today  ·  numbers from executed result files (single deterministic trials)')
probs = [
    ('1 · Gripper cannot get under the board', RED,
     ['Blank–table gap ≈ 0.2 mm vs 3 mm jaw pads',
      'Horizontal pinch: lower jaw overlaps table 41 mm; 20°/40° tilts 49/48 mm; 80 IK seeds: none collision-free',
      '2 pinch attempts: no opposing-jaw grasp, front crease < 1°, peak 24.6 N, 1.37 mm penetration']),
    ('2 · Commanded ≠ actual motion in contact', RED,
     ['Edge drag: 24.1 mm commanded → 13.1 mm hand travel; blank moved 0.42 mm',
      'Side push 10 mm → blank 0.44 mm (PD) / ≈ 4.7 mm (impedance), hand deflects up 2 mm',
      'Archived planner: joint tracking error 0.0205 rad > 0.02 guard at full speed; OK only at ½ speed']),
    ('3 · Contact forces hard to regulate', ORANGE,
     ['Peak force depends on speed: 4.84 N @ 5 mm/s → 0.61 N @ 0.5 mm/s',
      'Drag at ~3.5 N had a 13.3 N impact spike → emergency stop',
      'No force controller; impedance module is pose-based, not constant-force']),
    ('4 · Planner coverage', ORANGE,
     ['OMPL handles single-arm free space only',
      'No held-object, deliberate-contact or two-arm coordinated planning',
      'TrajOpt approach timed out (10 s); discrete, not continuous, collision checks']),
    ('5 · Model fidelity & success check', GREY,
     ['Rigid-panel approximation; crease spring/friction and μ = 0.6 are engineering guesses',
      'Tongue/slot geometry ≠ proven locking',
      'environment.py hard-codes succeed = False, physical_locking_verified = False']),
    ('6 · No agent in the loop yet', GREY,
     ['RAX is installed but not connected to the box sim',
      'Earlier Claude-teacher prototype archived (last cloud run: 19 requests, $0.74, no contact)',
      'Need: grasp / edge-lift / fold-about-hinge skills before agent exploration pays off']),
]
cw, ch = 4.1, 2.85
for k, (title, col, bullets) in enumerate(probs):
    x = 0.4 + (k % 3) * (cw + 0.12); y = 1.15 + (k // 3) * (ch + 0.12)
    box(s, x, y, cw, ch, LIGHT, None)
    box(s, x, y, 0.09, ch, col, None, shape=MSO_SHAPE.RECTANGLE)
    text(s, x + 0.18, y + 0.1, cw - 0.25, 0.4, title, size=15, bold=True, color=NAVY)
    tb = text(s, x + 0.18, y + 0.6, cw - 0.28, ch - 0.65, [[('•  ', {'color': col, 'bold': True}), (b, {})] for b in bullets],
         size=13)
    for p in tb.text_frame.paragraphs: p.space_after = Pt(7)
footer(s, 'Sources: validation/{feasibility_tester, edge_exposure, side_push, slow_contact, community_contact_tests, planner_comparison}; '
          'archived figures from ~/.local/share/box-folding-archive/20261103-230107 (commit c6f5352).')

prs.save(OUT / 'Dexmate_Box_Folding_Summary.pptx')
print('saved')
