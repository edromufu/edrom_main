import math

from bhv_simulator.trajectory_navigation import plan_path, path_velocity, ROBOT_RADIUS, MARGIN
from bhv_simulator.field_geometry import integrate_velocity
from bhv_simulator.thtastart_v2_p1inguin import weighted_theta_star, line_of_sight


def test_corner_cutting_and_blocked_endpoints():
    grid = [[1, -1], [-1, 1]]
    assert not line_of_sight(grid, (0, 0), (1, 1))
    assert weighted_theta_star(grid, (0, 0), (1, 1)) == []
    assert weighted_theta_star(grid, (1, 0), (1, 0)) == []


def test_detour_and_no_path_stop():
    path = plan_path((0, 0), (3, 0), [(1.1, 0, 0.2)])
    assert len(path) > 2
    assert path[0] == (0, 0) and path[-1] == (3, 0)
    assert plan_path((0, 0), (3, 0), [(0, 0, 0.2)]) == []
    assert path_velocity((0, 0), 0, (3, 0), []) == (0, 0)
    assert path_velocity((2.8, 0), 0, (3, 0), [(2.8, 0), (3, 0)]) == (0, 0)


def test_closed_loop_reaches_ball_without_intersecting_obstacles():
    obstacles = [(1.1, 0, 0.2), (2.1, 0.8, 0.2), (2.0, -0.9, 0.2)]
    x, y, yaw = 0, 0, -1.83254
    for _ in range(1600):
        path = plan_path((x, y), (3, 0), obstacles)
        assert path, (x, y)
        v, w = path_velocity((x, y), yaw, (3, 0), path)
        x, y, yaw = integrate_velocity(x, y, yaw, v, 0, w, 0.2)
        assert all(math.hypot(x - ox, y - oy) > r + ROBOT_RADIUS + MARGIN for ox, oy, r in obstacles)
        if math.hypot(x - 3, y) <= 0.251:
            break
    assert math.hypot(x - 3, y) <= 0.251
