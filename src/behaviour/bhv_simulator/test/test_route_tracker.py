import math
from pathlib import Path
from unittest.mock import patch

from bhv_simulator.trajectory_navigation import RouteTracker, path_velocity, ROBOT_RADIUS, MARGIN
from bhv_simulator.field_geometry import integrate_velocity
from bhv_simulator.planning.know import path_is_clear

OBSTACLES = [dict(x=1.1, y=0., raio=.2), dict(x=2.1, y=.8, raio=.2), dict(x=2., y=-.9, raio=.2)]


def test_tiny_ball_drift_does_not_repeat_theta():
    tracker = RouteTracker()
    with patch.object(tracker.planner, 'gerar_caminho_theta', wraps=tracker.planner.gerar_caminho_theta) as plan:
        for i in range(100):
            assert tracker.update((0., 0.), (3.+i*1e-6, 0.), OBSTACLES, i*.05)
        assert plan.call_count == 1
        assert tracker.update((0., 0.), (3.1, 0.), OBSTACLES, 5.)
        assert plan.call_count == 2


def test_new_obstacle_immediately_stops_even_during_replan_throttle():
    tracker = RouteTracker()
    assert tracker.update((0., 0.), (3., 0.), [], 0.)
    assert tracker.update((0., 0.), (3., 0.), [dict(x=.3, y=0., raio=.2)], .05) == []


def test_cached_route_reaches_ball_and_does_not_loop():
    tracker = RouteTracker()
    x, y, yaw = 0., 0., 0.
    yaw_travel = 0.
    for i in range(2400):
        path = tracker.update((x, y), (3., 0.), OBSTACLES, i*.05)
        v, w = tracker.command((x, y), yaw, (3., 0.), path, OBSTACLES)
        x, y, yaw = integrate_velocity(x, y, yaw, v, 0, w, .05)
        yaw_travel += abs(w*.05)
        assert path_is_clear([(x, y)], OBSTACLES, ROBOT_RADIUS+MARGIN)
        if math.dist((x, y), (3., 0.)) < .251:
            break
    assert math.dist((x, y), (3., 0.)) < .251, (x, y)
    assert yaw_travel < 2*math.pi, yaw_travel
    assert tracker.plan_count < 15, tracker.plan_count


def test_angular_error_wraps_at_pi():
    v, w = path_velocity((0., 0.), math.pi-.01, (-3., -.01), [(0., 0.), (-3., -.01)])
    assert v > 0
    assert abs(w) < .1


def test_world_body_is_kinematic():
    world = Path(__file__).parents[1] / 'worlds' / 'bhv_sim_world.wbt'
    body = world.read_text().split('DEF Robot3D Transform {')[1].split('DEF Obstacle1')[0]
    assert 'physics Physics' not in body
