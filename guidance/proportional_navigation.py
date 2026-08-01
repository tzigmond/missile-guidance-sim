import numpy as np


def proportional_navigation(missile_pos,
                            missile_vel,
                            target_pos,
                            target_vel,
                            N=5):
    """
    Proportional Navigation Guidance (3D)

    Commands lateral acceleration proportional to the Line-Of-Sight (LOS)
    rotation rate, causing the missile to steer toward a collision course.

    The standard PN law is:
        a_cmd = N * Vc * (omega x r_hat)

    where:
        N     = navigation constant (effective navigation ratio)
        Vc    = closing velocity (rate of range decrease)
        omega = LOS angular rate vector (rad/s)
        r_hat = unit LOS vector

    Note: the acceleration is formed by crossing omega with r_hat,
    NOT by using omega directly. omega points perpendicular to the LOS
    plane (like a torque axis), while (omega x r_hat) gives the actual
    in-plane steering direction. Using omega directly as the acceleration
    vector is a common implementation error that causes the missile to
    fly wildly out of plane in 3D scenarios.

    Ref: DTIC ADP010953 - Proportional Navigation guidance law
    Ref: MIL-HDBK-1211 (1995) - PN implementation

    PARAMETERS
    ----------
    missile_pos : np.ndarray (3,)
        Current missile position [m]
    missile_vel : np.ndarray (3,)
        Current missile velocity vector [m/s]
    target_pos : np.ndarray (3,)
        Current target position [m]
    target_vel : np.ndarray (3,)
        Current target velocity vector [m/s]
    N : float
        Effective navigation ratio (typically 3-5 for tail-chase,
        5 recommended for this scenario geometry)

    RETURNS
    -------
    accel_command : np.ndarray (3,)
        Commanded acceleration vector [m/s^2], to be projected
        laterally by the caller before application
    """

    # ----------------------------------------------------------
    # Step 1: LOS vector and unit vector
    #
    # r points from missile to target.
    # ----------------------------------------------------------
    r = target_pos - missile_pos
    dist = np.linalg.norm(r)
    r_hat = r / dist

    # ----------------------------------------------------------
    # Step 2: Relative velocity
    # ----------------------------------------------------------
    v_rel = target_vel - missile_vel

    # ----------------------------------------------------------
    # Step 3: LOS angular rate vector (rad/s)
    #
    # omega = (r x v_rel) / |r|^2
    #
    # This vector points perpendicular to the LOS plane (like a
    # rotation axis). Its magnitude is the angular rate of LOS
    # rotation. Do NOT use this directly as an acceleration - it
    # points out-of-plane.
    #
    # Ref: DTIC ADP010953
    # ----------------------------------------------------------
    omega = np.cross(r, v_rel) / dist**2

    # ----------------------------------------------------------
    # Step 4: Closing velocity
    #
    # Vc > 0 means missile and target are converging.
    # ----------------------------------------------------------
    closing_vel = -np.dot(r, v_rel) / dist

    # ----------------------------------------------------------
    # Step 5: Commanded acceleration
    #
    # a_cmd = N * Vc * (omega x r_hat)
    #
    # The cross product (omega x r_hat) rotates the axis-vector
    # omega back into the LOS plane, giving an in-plane steering
    # direction perpendicular to the LOS.
    #
    # Ref: DTIC ADP010953
    # ----------------------------------------------------------
    accel_command = N * closing_vel * np.cross(omega, r_hat)

    return accel_command