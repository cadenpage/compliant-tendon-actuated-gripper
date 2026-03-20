import argparse
import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
IMG_DIR = REPO_ROOT / "imgs"
DEFAULT_PLOT_PATH = IMG_DIR / "prb-analysis.png"

os.environ.setdefault("MPLCONFIGDIR", str(REPO_ROOT / ".cache" / "matplotlib"))


def add_local_venv():
    version_dir = "python{}.{}".format(sys.version_info.major, sys.version_info.minor)
    candidates = [
        REPO_ROOT / ".venv" / "lib" / version_dir / "site-packages",
        REPO_ROOT / ".venv" / "Lib" / "site-packages",
    ]
    for path in candidates:
        if path.exists():
            sys.path.insert(0, str(path))
            break


add_local_venv()

if "--show" not in sys.argv:
    import matplotlib

    matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


PARAMS = {
    "flexure_thickness_mm": 3.0,
    "strip_length_mm": 120.0,
    "face_angle_deg": 28.0,
    "link_lengths_mm": (34.0, 27.0, 21.0),
    "flexure_widths_mm": (12.0, 11.0, 10.0),
    "flexure_lengths_mm": (10.0, 9.0, 8.0),
    "tendon_moment_arms_mm": (7.0, 6.0, 5.0),
    "activation_torques_nmm": (2.0, 4.8, 8.0),
    "joint_limits_deg": (50.0, 58.0, 68.0),
    "tpu_modulus_mpa": 55.0,
    "strip_width_mm": 5.0,
    "strip_thickness_mm": 0.45,
    "effective_strip_modulus_mpa": 95.0,
    "distal_face_length_mm": 8.0,
}


def get_params(params=None):
    if params is None:
        return PARAMS
    return params


def torsional_stiffnesses(params=None):
    params = get_params(params)
    widths = np.asarray(params["flexure_widths_mm"], dtype=float)
    lengths = np.asarray(params["flexure_lengths_mm"], dtype=float)
    thickness = params["flexure_thickness_mm"]
    second_moment = widths * thickness**3 / 12.0
    return params["tpu_modulus_mpa"] * second_moment / lengths


def tendon_axial_stiffness(params=None):
    params = get_params(params)
    area_mm2 = params["strip_width_mm"] * params["strip_thickness_mm"]
    return params["effective_strip_modulus_mpa"] * area_mm2 / params["strip_length_mm"]


def joint_angles_from_tension(tension_n, params=None):
    params = get_params(params)
    radii = np.asarray(params["tendon_moment_arms_mm"], dtype=float)
    activation = np.asarray(params["activation_torques_nmm"], dtype=float)
    stiffness = torsional_stiffnesses(params)
    limits = np.radians(np.asarray(params["joint_limits_deg"], dtype=float))
    raw_angles = (tension_n * radii - activation) / stiffness
    return np.clip(raw_angles, 0.0, limits)


def pull_from_tension(tension_n, params=None):
    params = get_params(params)
    radii = np.asarray(params["tendon_moment_arms_mm"], dtype=float)
    tendon_stretch = tension_n / tendon_axial_stiffness(params)
    rotational_shortening = radii @ joint_angles_from_tension(tension_n, params)
    return float(tendon_stretch + rotational_shortening)


def solve_tension_for_pull(pull_mm, params=None):
    params = get_params(params)
    if pull_mm <= 0.0:
        return 0.0

    lower = 0.0
    upper = 1.0

    while pull_from_tension(upper, params) < pull_mm:
        upper *= 2.0
        if upper > 1000.0:
            break

    for _ in range(80):
        midpoint = 0.5 * (lower + upper)
        if pull_from_tension(midpoint, params) < pull_mm:
            lower = midpoint
        else:
            upper = midpoint

    return 0.5 * (lower + upper)


def joint_positions(joint_angles, params=None):
    params = get_params(params)
    points = [np.array([0.0, 0.0])]
    x_pos = 0.0
    y_pos = 0.0
    running_angle = 0.0

    for length_mm, angle_rad in zip(params["link_lengths_mm"], joint_angles):
        running_angle += angle_rad
        x_pos += length_mm * math.cos(running_angle)
        y_pos += length_mm * math.sin(running_angle)
        points.append(np.array([x_pos, y_pos]))

    return np.vstack(points)


def fingertip_state(pull_mm, params=None):
    params = get_params(params)
    tension_n = solve_tension_for_pull(pull_mm, params)
    joint_angles = joint_angles_from_tension(tension_n, params)
    points = joint_positions(joint_angles, params)
    face_orientation = math.degrees(float(np.sum(joint_angles))) + params["face_angle_deg"]
    return {
        "actuator_pull_mm": pull_mm,
        "tendon_tension_n": tension_n,
        "joint_angles_rad": joint_angles,
        "tip_xy_mm": points[-1],
        "face_orientation_deg": face_orientation,
    }


def sample_states(params=None, max_pull_mm=18.0, samples=160):
    params = get_params(params)
    pulls_mm = np.linspace(0.0, max_pull_mm, samples)
    states = [fingertip_state(pull_mm, params) for pull_mm in pulls_mm]
    return pulls_mm, states


def distal_face_segment(points, params=None):
    params = get_params(params)
    tip = points[-1]
    terminal_link_angle = math.atan2(
        points[-1, 1] - points[-2, 1],
        points[-1, 0] - points[-2, 0],
    )
    face_angle = terminal_link_angle + math.radians(params["face_angle_deg"])
    face_vector = 0.5 * params["distal_face_length_mm"] * np.array(
        [math.cos(face_angle), math.sin(face_angle)]
    )
    return np.vstack([tip - face_vector, tip + face_vector])


def plot_analysis(params=None, out_path=DEFAULT_PLOT_PATH, max_pull_mm=18.0, samples=160, show=False):
    params = get_params(params)
    pulls_mm, states = sample_states(params=params, max_pull_mm=max_pull_mm, samples=samples)
    angle_deg = np.degrees(np.array([state["joint_angles_rad"] for state in states]))
    tips_mm = np.array([state["tip_xy_mm"] for state in states])

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    figure, axes = plt.subplots(1, 3, figsize=(14, 4.6), constrained_layout=True)
    shape_axis, angle_axis, trajectory_axis = axes
    colors = plt.cm.viridis(np.linspace(0.1, 0.9, 5))

    for color, pull_mm in zip(colors, np.linspace(0.0, max_pull_mm, 5)):
        state = fingertip_state(float(pull_mm), params)
        points = joint_positions(state["joint_angles_rad"], params)
        face_segment = distal_face_segment(points, params)
        shape_axis.plot(points[:, 0], points[:, 1], color=color, linewidth=3)
        shape_axis.plot(face_segment[:, 0], face_segment[:, 1], color=color, linewidth=2, alpha=0.8)
        shape_axis.scatter(points[:, 0], points[:, 1], color=color, s=25)

    shape_axis.set_title("Equivalent 3R Finger Shapes")
    shape_axis.set_xlabel("x [mm]")
    shape_axis.set_ylabel("y [mm]")
    shape_axis.set_aspect("equal", adjustable="box")
    shape_axis.grid(alpha=0.25)

    for index, label in enumerate(("Proximal", "Middle", "Distal")):
        angle_axis.plot(pulls_mm, angle_deg[:, index], linewidth=2.5, label=label)
    angle_axis.set_title("Joint Angles vs. Actuator Pull")
    angle_axis.set_xlabel("Actuator pull [mm]")
    angle_axis.set_ylabel("Joint angle [deg]")
    angle_axis.grid(alpha=0.25)
    angle_axis.legend(frameon=False)

    marker_step = max(1, len(tips_mm) // 12)
    trajectory_axis.plot(tips_mm[:, 0], tips_mm[:, 1], color="#1f77b4", linewidth=3)
    trajectory_axis.scatter(
        tips_mm[::marker_step, 0],
        tips_mm[::marker_step, 1],
        color="#1f77b4",
        s=20,
    )
    trajectory_axis.set_title("Predicted Fingertip Path")
    trajectory_axis.set_xlabel("x [mm]")
    trajectory_axis.set_ylabel("y [mm]")
    trajectory_axis.set_aspect("equal", adjustable="box")
    trajectory_axis.grid(alpha=0.25)

    figure.suptitle("First-Pass PRB Model for a Tendon-Actuated Compliant Finger", fontsize=13)
    figure.savefig(out_path, dpi=220)
    if show:
        plt.show()
    plt.close(figure)
    return out_path


def summary_text(params=None, max_pull_mm=18.0):
    params = get_params(params)
    stiffness = torsional_stiffnesses(params)
    tendon_stiffness = tendon_axial_stiffness(params)
    final_state = fingertip_state(max_pull_mm, params)
    angle_deg = np.degrees(final_state["joint_angles_rad"])
    lines = [
        "Baseline PRB finger model",
        "  flexure thickness: {:.2f} mm".format(params["flexure_thickness_mm"]),
        "  strip length: {:.1f} mm".format(params["strip_length_mm"]),
        "  face angle: {:.1f} deg".format(params["face_angle_deg"]),
        "  torsional stiffnesses [N*mm/rad]: " + ", ".join("{:.2f}".format(value) for value in stiffness),
        "  tendon axial stiffness: {:.3f} N/mm".format(tendon_stiffness),
        "  max pull evaluated: {:.1f} mm".format(max_pull_mm),
        "  tendon tension at max pull: {:.3f} N".format(final_state["tendon_tension_n"]),
        "  joint angles at max pull [deg]: " + ", ".join("{:.2f}".format(value) for value in angle_deg),
        "  fingertip at max pull [mm]: ({:.2f}, {:.2f})".format(
            final_state["tip_xy_mm"][0],
            final_state["tip_xy_mm"][1],
        ),
        "  distal face orientation at max pull: {:.2f} deg".format(final_state["face_orientation_deg"]),
    ]
    return "\n".join(lines)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Generate the first-pass PRB analysis figure for the compliant finger."
    )
    parser.add_argument("--out", default=DEFAULT_PLOT_PATH, help="Path to save the generated analysis figure.")
    parser.add_argument("--max-pull", type=float, default=18.0, help="Maximum actuator pull to evaluate in millimeters.")
    parser.add_argument("--samples", type=int, default=160, help="Number of pull samples used for the plot.")
    parser.add_argument("--show", action="store_true", help="Display the figure after saving it.")
    return parser


def main():
    args = build_parser().parse_args()
    out_path = plot_analysis(
        params=PARAMS,
        out_path=args.out,
        max_pull_mm=args.max_pull,
        samples=args.samples,
        show=args.show,
    )
    print(summary_text(PARAMS, max_pull_mm=args.max_pull))
    print("Saved analysis figure to {}".format(out_path))


if __name__ == "__main__":
    main()
