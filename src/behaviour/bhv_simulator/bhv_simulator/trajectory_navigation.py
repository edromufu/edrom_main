"""Small adapter around the copied planner, and planar velocity tracking."""
import math

from .planning.know import Blackboard, HybridPlanner, TypeRef, path_is_clear
from .field_geometry import integrate_velocity

FIELD_BOUNDS = (-4.3, 4.3, -2.8, 2.8)
# World torso footprint: 0.20 x 0.11 m, circumscribed radius 0.114 m.
ROBOT_RADIUS, MARGIN = 0.12, 0.05
DEFAULT_APF = dict(k_atr=1.5, k_rep=0.08, d0=0.35, raio_robo=ROBOT_RADIUS,
                   velocidade_max=0.25, dt=0.05)


def plan_path(start, goal, obstacles):
    bb = Blackboard()
    bb.set(TypeRef.ME_POS, start)
    bb.set(TypeRef.BALL_POS, goal)
    bb.set(TypeRef.OBSTACLES, [dict(x=x, y=y, raio=r) for x, y, r in obstacles])
    return HybridPlanner(bb, DEFAULT_APF, limites=FIELD_BOUNDS,
                         margem_seguranca=MARGIN).gerar_caminho_theta()


def path_velocity(position, yaw, goal, path, max_linear=0.5, max_angular=1.2):
    if len(path) < 2 or math.dist(position, goal) <= 0.25:
        return 0.0, 0.0
    dx, dy = path[1][0] - position[0], path[1][1] - position[1]
    error = math.atan2(dy, dx) - yaw
    error = math.atan2(math.sin(error), math.cos(error))
    angular = max(-max_angular, min(max_angular, 2.5 * error))
    linear = min(max_linear, 2 * math.hypot(dx, dy),
                 2 * max(0, math.dist(position, goal) - 0.25))
    linear *= max(0, math.cos(error)) if abs(error) < 0.6 else 0
    return linear, angular


class RouteTracker:
    """Plan on scene changes/deviation, track cached waypoints otherwise."""
    def __init__(self, radius=ROBOT_RADIUS, margin=MARGIN):
        self.bb = Blackboard()
        params = dict(DEFAULT_APF, raio_robo=radius)
        self.planner = HybridPlanner(self.bb, params, limites=FIELD_BOUNDS,
                                     margem_seguranca=margin)
        self.clearance = radius + margin
        self.path, self.scene = [], None
        self.last_plan = float('-inf')
        self.plan_count = 0

    def clear(self):
        self.path, self.scene = [], None

    def update(self, start, goal, obstacles, now):
        scene = (tuple(goal), tuple(sorted((o['x'], o['y'], o['raio']) for o in obstacles)))
        changed = self.scene is None or math.dist(scene[0], self.scene[0]) > 0.03
        if self.scene is not None:
            changed |= len(scene[1]) != len(self.scene[1]) or any(
                math.dist(a, b) > 0.02 for a, b in zip(scene[1], self.scene[1]))
        # Advance only when the shortcut to the following waypoint is clear.
        while len(self.path) > 2 and math.dist(start, self.path[1]) < 0.1:
            if not path_is_clear([start, self.path[2]], obstacles, self.clearance, FIELD_BOUNDS):
                break
            self.path[0] = self.path.pop(1)
        remaining = [tuple(start)] + self.path[1:] if self.path else []
        valid = path_is_clear(remaining, obstacles, self.clearance, FIELD_BOUNDS)
        # Replan if pushed sideways more than 10 cm from the tracked segment.
        deviation = False
        if len(self.path) > 1:
            a, b = self.path[:2]
            dx, dy = b[0]-a[0], b[1]-a[1]
            d2 = dx*dx + dy*dy
            t = max(0, min(1, ((start[0]-a[0])*dx + (start[1]-a[1])*dy)/d2)) if d2 else 0
            deviation = math.dist(start, (a[0]+t*dx, a[1]+t*dy)) > 0.1
        needs_plan = changed or not valid or deviation
        if needs_plan and (now < self.last_plan or now - self.last_plan >= 0.5):
            self.bb.set(TypeRef.ME_POS, start)
            self.bb.set(TypeRef.BALL_POS, goal)
            self.bb.set(TypeRef.OBSTACLES, obstacles)
            self.path = self.planner.gerar_caminho_theta()
            self.scene, self.last_plan = scene, now
            self.plan_count += 1
            return self.path
        return [] if needs_plan else remaining

    def command(self, start, yaw, goal, path, obstacles, max_linear=0.5):
        v, w = path_velocity(start, yaw, goal, path, max_linear=max_linear)
        # Check the swept centre arc through the bridge's 0.3 s command timeout.
        arc = [integrate_velocity(*start, yaw, v, 0, w, t/100)[:2]
               for t in range(0, 31, 5)]
        if not path_is_clear(arc, obstacles, self.clearance, FIELD_BOUNDS):
            v = 0.0
        return v, w
