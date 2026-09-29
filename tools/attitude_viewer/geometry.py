"""FRD body to NED navigation attitude geometry."""


def rotation_matrix(q_nb: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], ...]:
    """Return the body-to-navigation matrix for a unit Hamilton quaternion."""
    w, x, y, z = q_nb
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )
