# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "scipy", "matplotlib"]
# ///
"""Plot GUAM simulation truth signals in separate figure windows.

Run from the repository root:
    uv run Scripts/plot_simout.py Exec_Scripts/SimOut_case6_python.mat
    uv run Scripts/plot_simout.py output.mat --plots pos vel att

Alternatively install numpy, scipy, matplotlib and use python3 instead of uv run.
For MATLAB timeseries objects, first run in MATLAB:
    addpath('Scripts')
    exportSimOutForPython('Exec_Scripts/SimOut_case6.mat')

Edit PLOTS below to add figures, choose fields/components, or change units.
Paths are relative to SimOut; component indices are ZERO-BASED in Python.
Each signal uses its own timestamps; no resampling or alignment is performed.
The default units assume GUAM's feet/slug/second configuration.
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np
from scipy.io import loadmat
from scipy.io.matlab import MatlabOpaque


@dataclass(frozen=True)
class Signal:
    path: str
    labels: tuple[str, ...]
    columns: tuple[int, ...] = ()  # Empty selects all components.
    scale: float = 1.0
    colors: tuple[str, ...] = ()


@dataclass(frozen=True)
class Plot:
    title: str
    ylabel: str
    signals: tuple[Signal, ...]


XYZ = ("r", "g", "b")
DEGREES = 180.0 / np.pi
INERTIAL = "Vehicle.EOM.InertialData"
EULER = "Vehicle.EOM.WorldRelativeData.Euler"
AERO = "Vehicle.FM.Aerodynamics"
THRUST = "Vehicle.FM.Propulsion.Fprop_r"

# One dictionary entry = one figure window. All fields are logged plant truth.
PLOTS = {
    "pos": Plot("Position — inertial axes", "Position (ft)", (
        Signal(f"{INERTIAL}.Pos_bii", ("X / North", "Y / East", "Z / Down"), colors=XYZ),
    )),
    "vel": Plot("Velocity — inertial axes", "Velocity (ft/s)", (
        Signal(f"{INERTIAL}.Vel_bIi", ("Vx", "Vy", "Vz"), colors=XYZ),
    )),
    "att": Plot("Attitude — local horizontal to body", "Angle (deg)", (
        Signal(f"{EULER}.phi", ("Roll",), scale=DEGREES, colors=("r",)),
        Signal(f"{EULER}.theta", ("Pitch",), scale=DEGREES, colors=("g",)),
        Signal(f"{EULER}.psi", ("Yaw",), scale=DEGREES, colors=("b",)),
    )),
    "ang_vel": Plot("Angular velocity — body axes, relative to inertial", "Rate (deg/s)", (
        Signal(f"{INERTIAL}.Omeg_BIb", ("p", "q", "r"), scale=DEGREES, colors=XYZ),
    )),
    "hover_thrust": Plot("Hover motor thrust — individual rotor axes", "Thrust (lbf)", (
        Signal(THRUST, tuple(f"Rotor {i}" for i in range(1, 9)), tuple(range(8))),
    )),
    "pusher_thrust": Plot("Pusher motor thrust — rotor axis", "Thrust (lbf)", (
        Signal(THRUST, ("Pusher (motor 9)",), (8,)),
    )),
    "aero_forces": Plot("Aerodynamic forces — body axes", "Force (lbf)", (
        Signal(f"{AERO}.Force_b", ("Fx", "Fy", "Fz"), colors=XYZ),
    )),
    "aero_moments": Plot("Aerodynamic moments — body axes", "Moment (lbf ft)", (
        Signal(f"{AERO}.Moment_b", ("L (roll)", "M (pitch)", "N (yaw)"), colors=XYZ),
    )),
}


def field(value, name):
    if isinstance(value, dict) and name in value:
        return value[name]
    if hasattr(value, "_fieldnames") and name in value._fieldnames:
        return getattr(value, name)
    raise KeyError(name)


def resolve(root, path):
    value = root
    for name in path.split("."):
        value = field(value, name)
    if isinstance(value, MatlabOpaque):
        raise ValueError(
            "MATLAB timeseries objects require conversion. In MATLAB run: "
            "addpath('Scripts'); exportSimOutForPython('your_file.mat')"
        )
    return value


def load_simout(path):
    path = Path(path)
    try:
        # Opening explicitly preserves filesystem errors that SciPy can mask
        # with "Reader needs file name or open file-like object" for Path inputs.
        with path.open("rb") as mat_file:
            contents = loadmat(mat_file, struct_as_record=False, squeeze_me=True)
    except FileNotFoundError as exc:
        message = f"MAT file not found: {path}."
        if path.stem.endswith("_python"):
            source = path.with_name(path.stem.removesuffix("_python") + path.suffix)
            if source.is_file():
                matlab_source = source.as_posix().replace("'", "''")
                message += (
                    " Create it first by running in MATLAB from the repository root: "
                    f"addpath('Scripts'); exportSimOutForPython('{matlab_source}')"
                )
        raise ValueError(message) from exc
    except NotImplementedError as exc:
        raise ValueError(
            "MATLAB v7.3 files require conversion with Scripts/exportSimOutForPython.m "
            "(which writes a SciPy-readable v7 file)."
        ) from exc
    if "SimOut" not in contents:
        raise ValueError("The MAT file must contain a variable named SimOut.")
    return contents["SimOut"]


def samples(root, path):
    """Return times and an N-sample by C-component array without resampling."""
    signal = resolve(root, path)
    try:
        data = np.asarray(field(signal, "Data"), dtype=float)
    except KeyError as exc:
        raise ValueError(f"{path} must contain Data and Time fields.") from exc
    try:
        time = np.asarray(field(signal, "Time"), dtype=float).reshape(-1)
    except KeyError:
        time = np.asarray(field(resolve(root, "Time"), "Data"), dtype=float).reshape(-1)

    if not len(time) or not np.all(np.isfinite(time)) or np.any(np.diff(time) < 0):
        raise ValueError(f"{path}: timestamps must be nonempty, finite, and nondecreasing.")
    data = np.squeeze(data)
    if data.ndim == 0:
        data = data.reshape(1, 1)
    elif data.ndim == 1:
        data = data.reshape(len(time), -1)
    else:
        try:
            time_first = bool(field(signal, "IsTimeFirst"))
        except KeyError:
            if data.shape[0] == len(time):
                time_first = True
            elif data.shape[-1] == len(time):
                time_first = False
            else:
                raise ValueError(f"{path}: cannot locate time axis in shape {data.shape}.")
        if not time_first:
            data = np.moveaxis(data, -1, 0)
        if data.shape[0] != len(time):
            raise ValueError(f"{path}: data and timestamp sample counts differ.")
        data = data.reshape(len(time), -1)
    if data.shape[0] != len(time):
        raise ValueError(f"{path}: data and timestamp sample counts differ.")
    return time, data


def make_figure(root, key, spec):
    # Read all fields before making a window so missing data skips a whole figure.
    series = []
    for signal in spec.signals:
        time, data = samples(root, signal.path)
        columns = signal.columns or tuple(range(data.shape[1]))
        if len(signal.labels) != len(columns):
            raise ValueError(f"{key}: labels and selected components must have equal lengths.")
        if any(c < 0 or c >= data.shape[1] for c in columns):
            raise ValueError(f"{signal.path}: requested columns {columns}, shape is {data.shape}.")
        values = data[:, columns] * signal.scale
        for i, label in enumerate(signal.labels):
            finite = values[:, i][np.isfinite(values[:, i])]
            bounds = f"{finite.min():.6g} .. {finite.max():.6g}" if len(finite) else "no finite values"
            print(f"{key}: SimOut.{signal.path}.Data[:, {columns[i]}] -> {label}: "
                  f"n={len(time)}, t={time[0]:.6g}..{time[-1]:.6g} s, "
                  f"range={bounds}, nonfinite={len(time)-len(finite)} (scale={signal.scale:g})")
        series.append((signal, time, values))

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(num=spec.title, figsize=(10, 5), layout="constrained")
    for signal, time, values in series:
        for i, label in enumerate(signal.labels):
            color = {"color": signal.colors[i]} if signal.colors else {}
            ax.plot(time, values[:, i], label=label, **color)
    ax.set(title=spec.title, xlabel="Time (s)", ylabel=spec.ylabel)
    ax.grid(True, alpha=0.35)
    ax.legend()
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mat_file", type=Path)
    parser.add_argument("--plots", nargs="+", choices=tuple(PLOTS), default=list(PLOTS))
    parser.add_argument("--save-dir", type=Path, help="Also save each figure as a PNG.")
    parser.add_argument("--no-show", action="store_true", help="Use a noninteractive backend (for batch export).")
    args = parser.parse_args()
    if args.no_show:
        import matplotlib
        matplotlib.use("Agg")
    try:
        root = load_simout(args.mat_file)
        # Fail early with conversion instructions for the standard MATLAB-object log.
        resolve(root, "Time")
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f"Error: {exc}\n")

    if args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)
    count = 0
    for key in dict.fromkeys(args.plots):
        try:
            fig = make_figure(root, key, PLOTS[key])
        except KeyError as exc:
            print(f"Skipping {key}: missing field {exc}. This signal must be included in the MATLAB log.", file=sys.stderr)
            continue
        except ValueError as exc:
            parser.exit(1, f"Error: {exc}\n")
        count += 1
        if args.save_dir:
            fig.savefig(args.save_dir / f"{key}.png", dpi=150)
    if not count:
        parser.exit(1, "No requested signals were available.\n")
    import matplotlib.pyplot as plt
    if not args.no_show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()
