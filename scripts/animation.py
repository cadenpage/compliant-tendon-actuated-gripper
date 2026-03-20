import argparse
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
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

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from prb import PARAMS, REPO_ROOT, distal_face_segment, fingertip_state, joint_positions

DEFAULT_GIF_PATH = REPO_ROOT / "imgs" / "finger-curl.gif"


def build_parser():
    parser = argparse.ArgumentParser(
        description="Animate the first-pass PRB finger model over actuator pull."
    )
    parser.add_argument("--out", default=DEFAULT_GIF_PATH, help="Path to save the generated GIF.")
    parser.add_argument("--max-pull", type=float, default=18.0, help="Maximum actuator pull to animate in millimeters.")
    parser.add_argument("--frames", type=int, default=60, help="Number of frames in the saved animation.")
    parser.add_argument("--fps", type=int, default=18, help="Frame rate for the saved animation.")
    parser.add_argument("--show", action="store_true", help="Display the animation window after saving it.")
    return parser


def main():
    args = build_parser().parse_args()
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    pull_values = np.linspace(0.0, args.max_pull, args.frames)
    states = [fingertip_state(float(pull_mm), PARAMS) for pull_mm in pull_values]
    shapes = [joint_positions(state["joint_angles_rad"], PARAMS) for state in states]
    faces = [distal_face_segment(points, PARAMS) for points in shapes]
    tip_trace = np.array([state["tip_xy_mm"] for state in states])

    total_length = sum(PARAMS["link_lengths_mm"])
    figure, axis = plt.subplots(figsize=(6.4, 5.2), constrained_layout=True)
    axis.set_title("Predicted Curling from Tendon Pull")
    axis.set_xlabel("x [mm]")
    axis.set_ylabel("y [mm]")
    axis.set_xlim(-5.0, total_length + 8.0)
    axis.set_ylim(-3.0, total_length * 0.8)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.25)

    axis.plot([0.0, 0.0], [-6.0, 10.0], color="black", linewidth=2.0, alpha=0.5)
    trajectory_line, = axis.plot([], [], color="#6baed6", linewidth=1.6, alpha=0.85)
    finger_line, = axis.plot([], [], color="#08306b", linewidth=3.2)
    face_line, = axis.plot([], [], color="#d94801", linewidth=2.4)
    joint_markers = axis.scatter([], [], s=26, color="#08306b")
    state_text = axis.text(
        0.03,
        0.96,
        "",
        transform=axis.transAxes,
        ha="left",
        va="top",
        fontsize=9,
        family="monospace",
        bbox={"facecolor": "white", "edgecolor": "#cccccc", "alpha": 0.9},
    )

    def update(frame_index):
        shape = shapes[frame_index]
        face = faces[frame_index]
        state = states[frame_index]

        finger_line.set_data(shape[:, 0], shape[:, 1])
        face_line.set_data(face[:, 0], face[:, 1])
        trajectory_line.set_data(tip_trace[: frame_index + 1, 0], tip_trace[: frame_index + 1, 1])
        joint_markers.set_offsets(shape)
        state_text.set_text(
            "\n".join(
                [
                    "pull = {:4.1f} mm".format(state["actuator_pull_mm"]),
                    "tension = {:4.2f} N".format(state["tendon_tension_n"]),
                    "angles = "
                    + ", ".join("{:4.1f}".format(value) for value in np.degrees(state["joint_angles_rad"]))
                    + " deg",
                ]
            )
        )
        return finger_line, face_line, trajectory_line, joint_markers, state_text

    movie = animation.FuncAnimation(
        figure,
        update,
        frames=len(shapes),
        interval=1000 / max(args.fps, 1),
        blit=False,
    )
    movie.save(out_path, writer=animation.PillowWriter(fps=args.fps))
    if args.show:
        plt.show()
    plt.close(figure)
    print("Saved finger curling animation to {}".format(out_path))


if __name__ == "__main__":
    main()
