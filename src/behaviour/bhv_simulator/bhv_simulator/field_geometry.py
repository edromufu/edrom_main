"""Conversions for the NUE Webots world and the ROS field frame (SI units)."""

import math


def field_position(position):
    """Map Webots (x, y-up, z) to field (-z, -x, y-up)."""
    x, y, z = position
    return -z, -x, y


def field_yaw(orientation):
    """Project Robot3D's forward axis (-local Z) into the field plane."""
    return math.atan2(orientation[2], orientation[8])


def integrate_velocity(x, y, yaw, vx, vy, omega, dt):
    """Integrate a constant body-frame planar twist, including circular arcs."""
    angle = omega * dt
    if abs(angle) < 1e-9:
        dx, dy = vx * dt, vy * dt
    else:
        a, b = math.sin(angle) / omega, (1 - math.cos(angle)) / omega
        dx, dy = a * vx - b * vy, b * vx + a * vy
    return (x + math.cos(yaw) * dx - math.sin(yaw) * dy,
            y + math.sin(yaw) * dx + math.cos(yaw) * dy,
            math.atan2(math.sin(yaw + angle), math.cos(yaw + angle)))
