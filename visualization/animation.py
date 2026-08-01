import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================================================
# Color palette - light bone theme
# ============================================================
BG_COLOR        = "#f5efe0"   # near-bone background
PANEL_COLOR     = "#ede4d0"   # axis panel face
GRID_COLOR      = "#c8b89a"   # subtle grid lines
TEXT_COLOR      = "#958373"   # light gray text
STATUS_COLOR    = '#ff4444'   # bright red for INTERCEPT

AIRCRAFT_COLOR  = '#c0392b'   # crimson - aircraft
MISSILE_COLOR   = '#e67e22'   # vivid orange - missile
START_AIRCRAFT  = "#2e7d32"   # forest green - aircraft start marker
START_MISSILE   = '#d35400'   # deep orange - missile start marker
INTERCEPT_COLOR = '#c0392b'   # dark red star - intercept marker

# # ============================================================
# # Color palette - dark tactical theme
# # ============================================================
# BG_COLOR        = '#0d0f14'   # near-black with cool blue tint
# PANEL_COLOR     = '#111520'   # deep navy panel
# GRID_COLOR      = '#1e2535'   # subtle dark grid
# TEXT_COLOR      = '#7a8fa6'   # muted steel blue-gray text
# STATUS_COLOR    = '#ff4444'   # bright red for INTERCEPT

# AIRCRAFT_COLOR  = '#00cfff'   # electric cyan - aircraft
# MISSILE_COLOR   = '#ff6a00'   # vivid orange - missile
# START_AIRCRAFT  = '#00cc66'   # bright green - aircraft start marker
# START_MISSILE   = '#ffaa00'   # amber - missile start marker
# INTERCEPT_COLOR = '#ff2222'   # red star - intercept marker

def animate(times, target_states, missile_states, result):
    """
    3D animated visualization of missile and target trajectories.

    All internal physics remain in metres. Display is converted to
    kilometres for readability.

    Parameters
    ----------
    times : np.ndarray
        Time array from simulation
    target_states : np.ndarray (N, 3)
        Target position history [m]
    missile_states : np.ndarray (N, 3)
        Missile position history [m]
    result : SimResult
        Simulation result dataclass containing intercept info
    """

    # ----------------------------------------------------------
    # Convert to km for display only
    # All internal physics remain in metres
    # ----------------------------------------------------------
    ts = target_states  / 1000.0
    ms = missile_states / 1000.0

    # ----------------------------------------------------------
    # Find the frame index of intercept (if any)
    # ----------------------------------------------------------
    intercept_frame = None
    if result.intercepted:
        intercept_time = result.intercept_time
        intercept_frame = int(np.argmin(np.abs(times - intercept_time)))

    # ----------------------------------------------------------
    # Figure and axes - dark background
    # ----------------------------------------------------------
    fig = plt.figure(figsize=(14, 10), facecolor=BG_COLOR)
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_facecolor(PANEL_COLOR)

    # Axis label and tick colors
    for axis in [ax.xaxis, ax.yaxis, ax.zaxis]:
        axis.label.set_color(TEXT_COLOR)
        axis._axinfo['tick']['color'] = TEXT_COLOR
        axis._axinfo['grid']['color'] = GRID_COLOR
        axis.pane.fill = False
        axis.pane.set_edgecolor(GRID_COLOR)

    ax.tick_params(colors=TEXT_COLOR)

    # ----------------------------------------------------------
    # Set axis limits centered on both trajectories (in km)
    # ----------------------------------------------------------
    if intercept_frame is not None:
        all_points = np.vstack([ts[:intercept_frame+1], ms[:intercept_frame+1]])  
    else:
        all_points = np.vstack([ts, ms])

    padding = 0.1
    max_range = max(
        np.ptp(all_points[:, 0]),
        np.ptp(all_points[:, 1]),
        np.ptp(all_points[:, 2])
    )
    x_center = (all_points[:, 0].max() + all_points[:, 0].min()) / 2
    y_center = (all_points[:, 1].max() + all_points[:, 1].min()) / 2
    z_center = (all_points[:, 2].max() + all_points[:, 2].min()) / 2
    plot_radius = max_range / 2 * (1 + padding)

    ax.set_xlim(x_center - plot_radius, x_center + plot_radius)
    ax.set_ylim(y_center - plot_radius, y_center + plot_radius)
    ax.set_zlim(z_center - plot_radius, z_center + plot_radius)
    ax.set_box_aspect([1, 1, 1])

    ax.set_xlabel('X (km)', labelpad=10)
    ax.set_ylabel('Y (km)', labelpad=10)
    ax.set_zlabel('Z (km)', labelpad=10)
    ax.set_title(
        '3D Missile-Aircraft Pursuit Simulation - Proportional Navigation',
        color=TEXT_COLOR,
        pad=15,
        fontsize=13
    )
    ax.grid(True, color=GRID_COLOR, alpha=0.5)
    ax.view_init(elev=20, azim=45)

    # ----------------------------------------------------------
    # Static markers for start positions (in km)
    # ----------------------------------------------------------
    ax.scatter(*ts[0], c=START_AIRCRAFT, s=120, marker='s',
               label='Aircraft Start', zorder=5, edgecolors='white', linewidths=0.5)
    ax.scatter(*ms[0], c=START_MISSILE,  s=120, marker='^',
               label='Missile Start',  zorder=5, edgecolors='white', linewidths=0.5)

    # ----------------------------------------------------------
    # Intercept marker - initially hidden
    # ----------------------------------------------------------
    intercept_marker, = ax.plot([], [], [], '*',
                                color=INTERCEPT_COLOR,
                                markersize=22,
                                label='Intercept',
                                zorder=10)

    # ----------------------------------------------------------
    # Animated artists
    # ----------------------------------------------------------
    target_point,  = ax.plot([], [], [], 'o',
                             color=AIRCRAFT_COLOR, markersize=10,
                             label='Aircraft', zorder=6)
    target_trail,  = ax.plot([], [], [], '-',
                             color=AIRCRAFT_COLOR, linewidth=2, alpha=0.7)

    missile_point, = ax.plot([], [], [], 'o',
                             color=MISSILE_COLOR, markersize=8,
                             label='Missile', zorder=6)
    missile_trail, = ax.plot([], [], [], '-',
                             color=MISSILE_COLOR, linewidth=1.5, alpha=0.7)

    # ----------------------------------------------------------
    # HUD text
    # ----------------------------------------------------------
    time_text     = ax.text2D(0.02, 0.95, '', transform=ax.transAxes,
                              fontsize=12, color=TEXT_COLOR, fontfamily='monospace')
    distance_text = ax.text2D(0.02, 0.90, '', transform=ax.transAxes,
                              fontsize=10, color=TEXT_COLOR, fontfamily='monospace')
    status_text   = ax.text2D(0.02, 0.85, '', transform=ax.transAxes,
                              fontsize=11, color=STATUS_COLOR,
                              fontweight='bold', fontfamily='monospace')

    # Legend styling
    ax.legend(facecolor='#1a1a1a', edgecolor=GRID_COLOR,
              labelcolor=TEXT_COLOR, fontsize=9)

    # ----------------------------------------------------------
    # Init
    # ----------------------------------------------------------
    def init():
        for artist in (target_point, target_trail, missile_point,
                       missile_trail, intercept_marker):
            artist.set_data([], [])
            artist.set_3d_properties([])
        time_text.set_text('')
        distance_text.set_text('')
        status_text.set_text('')
        return (target_point, target_trail, missile_point, missile_trail,
                intercept_marker, time_text, distance_text, status_text)

    # ----------------------------------------------------------
    # Update
    # ----------------------------------------------------------
    def update(frame):
        if intercept_frame is not None and frame >= intercept_frame:
            frozen = intercept_frame
        else:
            frozen = frame

        # Aircraft
        target_point.set_data([ts[frozen, 0]], [ts[frozen, 1]])
        target_point.set_3d_properties([ts[frozen, 2]])
        target_trail.set_data(ts[:frozen+1, 0], ts[:frozen+1, 1])
        target_trail.set_3d_properties(ts[:frozen+1, 2])

        # Missile
        missile_point.set_data([ms[frozen, 0]], [ms[frozen, 1]])
        missile_point.set_3d_properties([ms[frozen, 2]])
        missile_trail.set_data(ms[:frozen+1, 0], ms[:frozen+1, 1])
        missile_trail.set_3d_properties(ms[:frozen+1, 2])

        # Intercept marker
        if intercept_frame is not None and frame >= intercept_frame:
            ix, iy, iz = ts[intercept_frame]
            intercept_marker.set_data([ix], [iy])
            intercept_marker.set_3d_properties([iz])
            status_text.set_text('[ INTERCEPT ]')
        else:
            intercept_marker.set_data([], [])
            intercept_marker.set_3d_properties([])
            status_text.set_text('')

        # HUD
        distance_m   = np.linalg.norm(target_states[frozen] - missile_states[frozen])
        display_time = (times[intercept_frame]
                        if (intercept_frame is not None and frame >= intercept_frame)
                        else times[frame])
        time_text.set_text(f'T = {display_time:.2f} s')
        if distance_m < 1000:
            distance_text.set_text(f'D = {distance_m:.0f} m')
        else:
            distance_text.set_text(f'D = {distance_m/1000:.2f} km')


        return (target_point, target_trail, missile_point, missile_trail,
                intercept_marker, time_text, distance_text, status_text)

    frame_skip = max(1, len(times) // 500)
    frames     = range(0, len(times), frame_skip)

    anim = FuncAnimation(fig, update, frames=frames, init_func=init,
                         blit=False, interval=5, repeat=True)

    plt.tight_layout()
    plt.show()