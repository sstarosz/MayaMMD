"""Interactive 2D MMD attachment explainer. Standard-library Python + Tk.

This is a position-projection demonstration, not the Bullet dynamics engine.
Only translation is locked; relative rotation is free. Units are canvas pixels.
Run with --self-test for headless checks of the attachment and correction math.
"""

import math
import sys
import tkinter as tk
from dataclasses import dataclass
from tkinter import ttk


def rotate(p, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c * p[0] - s * p[1], s * p[0] + c * p[1]


@dataclass
class Body:
    x: float
    y: float
    angle: float = 0.0
    local: tuple = (0.0, 0.0)
    fixed: bool = False

    def bind(self, joint):
        self.local = rotate((joint[0] - self.x, joint[1] - self.y), -self.angle)

    def attachment(self):
        rx, ry = rotate(self.local, self.angle)
        return self.x + rx, self.y + ry


def correct(a, b, fraction=0.3):
    """Project coincident-pivot error, allowing COM motion and rotation.

    K = sum(inv_mass * I + inv_inertia * perp(r) * perp(r)^T).
    Equal unit masses; rectangle inertia for 180 x 54 bodies.
    Linearized angular corrections are iterated, rather than solved at once.
    """
    pa, pb = a.attachment(), b.attachment()
    error = (pb[0] - pa[0], pb[1] - pa[1])
    kxx = kxy = kyy = 0.0
    terms = []
    for body in (a, b):
        im = 0.0 if body.fixed else 1.0
        ii = im * 12.0 / (180.0**2 + 54.0**2)
        rx, ry = rotate(body.local, body.angle)
        terms.append((im, ii, -ry, rx))
        kxx += im + ii * ry * ry
        kxy -= ii * rx * ry
        kyy += im + ii * rx * rx
    det = kxx * kyy - kxy * kxy
    if det < 1e-12:
        return
    lx = fraction * (kyy * error[0] - kxy * error[1]) / det
    ly = fraction * (kxx * error[1] - kxy * error[0]) / det
    for body, (im, ii, px, py), sign in zip((a, b), terms, (1, -1)):
        body.x += sign * im * lx
        body.y += sign * im * ly
        body.angle += sign * ii * (px * lx + py * ly)


class Playground:
    def __init__(self, root):
        self.root = root
        root.title('MMD Joint Playground — initial frame and moving attachments')
        root.geometry('1120x860')
        self.bound = False
        self.running = False
        self.drag = None
        self.joint = (500.0, 350.0)
        self.a = Body(310, 350, fixed=True)
        self.b = Body(690, 350)
        ttk.Label(root, text='One initial joint → two body-local attachments',
                  font=('Segoe UI', 19, 'bold')).pack(pady=(12, 4))
        self.instruction = tk.StringVar()
        ttk.Label(root, textvariable=self.instruction, wraplength=1060).pack(pady=5)
        bar = ttk.Frame(root)
        bar.pack(pady=5)
        for label, action in [('Joint in middle', lambda: self.reset('middle')),
                              ('Joint inside A', lambda: self.reset('inside')),
                              ('Joint far away', lambda: self.reset('far')),
                              ('1. Capture attachments', self.capture),
                              ('2. Separate bodies', self.separate),
                              ('3. Correct slowly', self.play),
                              ('Pause', self.pause)]:
            ttk.Button(bar, text=label, command=action).pack(side='left', padx=3)
        options = ttk.Frame(root)
        options.pack()
        self.fixed = tk.BooleanVar(value=True)
        ttk.Checkbutton(options, text='A is kinematic (solver moves only B)',
                        variable=self.fixed, command=self.set_fixed).pack(side='left', padx=10)
        self.turn = tk.BooleanVar(value=True)
        ttk.Checkbutton(options, text='Include rotation when correcting',
                        variable=self.turn).pack(side='left', padx=10)
        ttk.Label(options, text='Translation locked • Rotation free').pack(side='left', padx=10)
        self.canvas = tk.Canvas(root, background='#f7f9fc', height=475,
                                highlightthickness=0)
        self.canvas.pack(fill='both', expand=True, padx=12, pady=8)
        self.canvas.bind('<ButtonPress-1>', self.press)
        self.canvas.bind('<B1-Motion>', self.move)
        self.canvas.bind('<ButtonRelease-1>', lambda _: setattr(self, 'drag', None))
        turns = ttk.Frame(root)
        turns.pack()
        for name, body in [('Rotate A −15°', self.a), ('Rotate B −15°', self.b)]:
            ttk.Button(turns, text=name,
                       command=lambda b=body: self.rotate_body(b, -15)).pack(side='left', padx=5)
        for name, body in [('Rotate A +15°', self.a), ('Rotate B +15°', self.b)]:
            ttk.Button(turns, text=name,
                       command=lambda b=body: self.rotate_body(b, 15)).pack(side='left', padx=5)
        self.readout = tk.StringVar()
        ttk.Label(root, textvariable=self.readout, font=('Consolas', 11),
                  justify='left').pack(pady=8)
        ttk.Label(root, text='Dashed lines are offsets, not physical rods. Attachments can be outside the colliders.\n'
                  'Educational 2D position solver, not Bullet. No gravity, collision, velocity, springs or angular limits.',
                  justify='center').pack(pady=(0, 12))
        self.reset('middle')

    def set_fixed(self):
        self.a.fixed = self.fixed.get()
        self.draw()

    def reset(self, preset):
        self.pause()
        self.bound = False
        self.a.x, self.a.y, self.a.angle = 310, 270, 0
        self.b.x, self.b.y, self.b.angle = 690, 270, 0
        self.joint = {'middle': (500, 270), 'inside': (270, 270),
                      'far': (500, 80)}[preset]
        self.a.local = self.b.local = (0, 0)
        self.draw()

    def capture(self):
        self.pause()
        if self.bound:
            return
        self.a.bind(self.joint)
        self.b.bind(self.joint)
        self.bound = True
        self.draw()

    def separate(self):
        if not self.bound:
            self.capture()
        self.pause()
        self.b.x += 65
        self.b.y += 55
        self.b.angle += math.radians(12)
        self.draw()

    def rotate_body(self, body, degrees):
        self.pause()
        body.angle += math.radians(degrees)
        self.draw()

    def pause(self):
        self.running = False

    def play(self):
        if not self.bound:
            self.capture()
        if not self.running:
            self.running = True
            self.tick()

    def tick(self):
        if not self.running:
            return
        if self.turn.get():
            correct(self.a, self.b, 0.12)
        else:
            pa, pb = self.a.attachment(), self.b.attachment()
            weight = 1.0 if self.a.fixed else 0.5
            dx, dy = (pb[0] - pa[0]) * 0.12, (pb[1] - pa[1]) * 0.12
            if not self.a.fixed:
                self.a.x += dx * weight
                self.a.y += dy * weight
            self.b.x -= dx * weight
            self.b.y -= dy * weight
        self.draw()
        if math.dist(self.a.attachment(), self.b.attachment()) < 0.05:
            self.running = False
        else:
            self.root.after(30, self.tick)

    def press(self, event):
        self.pause()
        if not self.bound and math.dist((event.x, event.y), self.joint) < 22:
            self.drag = ('joint', 0, 0)
            return
        for body in (self.a, self.b):
            u, v = rotate((event.x - body.x, event.y - body.y), -body.angle)
            if abs(u) <= 90 and abs(v) <= 27:
                self.drag = (body, event.x - body.x, event.y - body.y)
                return

    def move(self, event):
        if self.drag is None:
            return
        target, dx, dy = self.drag
        if target == 'joint':
            self.joint = (event.x, event.y)
        else:
            target.x, target.y = event.x - dx, event.y - dy
        self.draw()

    def draw(self):
        c = self.canvas
        c.delete('all')
        for x in range(0, 1600, 50):
            c.create_line(x, 0, x, 900, fill='#e8edf3')
        for y in range(0, 900, 50):
            c.create_line(0, y, 1600, y, fill='#e8edf3')
        jx, jy = self.joint
        c.create_line(jx - 12, jy, jx + 12, jy, fill='#8b5cf6', width=3)
        c.create_line(jx, jy - 12, jx, jy + 12, fill='#8b5cf6', width=3)
        c.create_text(jx, jy - 25, text='Stored initial joint', fill='#7042c1')
        for body, color, name in [(self.a, '#2283cc', 'A'), (self.b, '#e28a20', 'B')]:
            points = []
            for corner in [(-90, -27), (90, -27), (90, 27), (-90, 27)]:
                x, y = rotate(corner, body.angle)
                points.extend((body.x + x, body.y + y))
            c.create_polygon(points, fill=color, outline='#334155', width=2)
            c.create_oval(body.x - 4, body.y - 4, body.x + 4, body.y + 4, fill='white')
            c.create_text(body.x, body.y + 45,
                          text=f'{name}: center / COM' + (' (kinematic)' if body.fixed else ''),
                          fill=color, font=('Segoe UI', 11, 'bold'))
            if self.bound:
                ax, ay = body.attachment()
                c.create_line(body.x, body.y, ax, ay, fill=color, dash=(5, 4), width=2)
                radius = 13 if name == 'A' else 8
                c.create_oval(ax-radius, ay-radius, ax+radius, ay+radius,
                              outline=color, width=3)
                c.create_text(ax, ay + (23 if name == 'A' else 40),
                              text=f'Attachment {name}', fill=color)
        if self.bound:
            pa, pb = self.a.attachment(), self.b.attachment()
            c.create_line(*pa, *pb, fill='#dc3545', dash=(3, 3), width=2)
            self.instruction.set('Drag either body or use rotation buttons. Local attachments stay fixed. '
                                 'Correct slowly brings the current attachments together, not back to the purple cross.')
            self.readout.set(f'Initial joint: ({jx:.1f}, {jy:.1f})   Attachment gap: {math.dist(pa, pb):.2f}\n'
                             f'A local: ({self.a.local[0]:.1f}, {self.a.local[1]:.1f})   '
                             f'A world: ({pa[0]:.1f}, {pa[1]:.1f})\n'
                             f'B local: ({self.b.local[0]:.1f}, {self.b.local[1]:.1f})   '
                             f'B world: ({pb[0]:.1f}, {pb[1]:.1f})')
        else:
            self.instruction.set('SETUP: drag the purple joint anywhere, including outside both bodies. '
                                 'Drag bodies to arrange them. Then capture attachments.')
            self.readout.set('Offsets are measured in each body\'s local frame, whose origin is the center / COM here.\n'
                             'The endpoint or collider surface has no special meaning.\n'
                             'Choose a preset to return to setup; captured attachments cannot slide.')


def self_test():
    for joint in [(500, 270), (270, 270), (500, 80)]:
        for fixed in (True, False):
            a, b = Body(310, 270, 0.4, fixed=fixed), Body(690, 270, -0.2)
            a.bind(joint)
            b.bind(joint)
            assert math.dist(a.attachment(), joint) < 1e-9
            assert math.dist(b.attachment(), joint) < 1e-9
            locals_before = (a.local, b.local)
            pose_a = (a.x, a.y, a.angle)
            b.x += 65
            b.y += 55
            b.angle += 0.2
            for _ in range(250):
                correct(a, b)
            assert math.dist(a.attachment(), b.attachment()) < 1e-6
            assert (a.local, b.local) == locals_before
            if fixed:
                assert (a.x, a.y, a.angle) == pose_a
    print('PASS: bind transforms, inside/outside pivots, convergence, fixed A, constant local attachments')


if __name__ == '__main__':
    if '--self-test' in sys.argv:
        self_test()
    else:
        app = tk.Tk()
        Playground(app)
        app.mainloop()
