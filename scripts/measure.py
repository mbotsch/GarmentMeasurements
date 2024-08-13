from argparse import ArgumentParser
import os
from pathlib import Path
import re
import subprocess


CONFIG = {
    "EXE": "",
    "DATA_DIR": "",
    "MESHES_DIR": "meshes",
    "MEASUREMENT_DIR": "measurements",
    "LOG_DIR": "logs",
}


def measure_mesh(
    mesh_file: Path, yaml_file: Path,
    write_measurements: bool = False, stdout: bool = False, update: bool = False
):
    # do not measure if measurement file exists
    if not update and yaml_file.exists():
        return

    cmd_line = [str(CONFIG["EXE"]), str(mesh_file), str(yaml_file)]

    if stdout:
        cmd_line.append("--stdout")

    if write_measurements:
        cmd_line.append("--debug")
        cmd_line.append("--debug_dir")
        cmd_line.append(str(Path(CONFIG["LOG_DIR"])))

    pipes = subprocess.Popen(cmd_line, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    std_out, std_err = pipes.communicate()
    std_out = std_out.decode("utf-8").strip()
    std_err = std_err.decode("utf-8").strip()

    if pipes.returncode != 0:
        raise Exception(f"Error code {pipes.returncode}.")

    if std_err != "":
        print(f"[!!!] {std_err}")

    if std_out != "":
        print(std_out)


def list_mesh_files(filepath: Path) -> list[str]:
    regex = re.compile(r"(\d+_straight\.obj)$")

    # find all files in mesh directory
    files = [p for p in filepath.iterdir() if p.is_file()]
    # get filename
    filenames = [p.name for p in files]
    # match all numbers in filename
    filenames = [p for p in filenames if re.match(regex, p)]
    return filenames


def configure(args):
    # cwd = os.getcwd()
    CONFIG["EXE"] = args.executable
    CONFIG["DATA_DIR"] = args.data_dir
    CONFIG["MESHES_DIR"] = args.meshes_dir
    CONFIG["MEASUREMENT_DIR"] = args.measurement_dir
    CONFIG["LOG_DIR"] = args.log_dir


def main():
    parser = ArgumentParser("Measure caesar data set")
    parser.add_argument("--executable", type=str,
                        help="Path to measurement executable.",
                        default=Path(os.getcwd()).parent / "build" / "measurements")
    parser.add_argument("--data_dir", type=Path,
                        help="Directory to caesar files.",
                        default=Path(os.getcwd()).parent / "data")
    parser.add_argument("--meshes_dir", type=Path,
                        help="Directory of mesh files",
                        default=Path(os.getcwd()).parent / "output" / "meshes")
    parser.add_argument("--measurement_dir", type=Path,
                        help="Directory for measurement files.",
                        default=Path(os.getcwd()).parent / "output" / "measurements")
    parser.add_argument("--log_dir", type=Path,
                        help="Debugging log directory.",
                        default=Path(os.getcwd()).parent / "output" / "logs")

    args, extra_args = parser.parse_known_args()
    configure(args)

    measurement_dir = Path(CONFIG["MEASUREMENT_DIR"])
    meshes_dir = Path(CONFIG["MESHES_DIR"])

    meshes = list_mesh_files(meshes_dir)

    # create ouput dir
    measurement_dir.mkdir(exist_ok=True)

    # number match
    number_re = re.compile(r'\d+')

    # If some error occurs, measure this object only
    # meshes = ['00089_apart.obj']
    debug_mode = False  # True for saving measurements

    for mesh in meshes:  # [:3] first three
        numbers = re.findall(number_re, mesh)
        if len(numbers) != 1:
            continue

        print(f"Process {mesh}")
        measure_mesh(meshes_dir / mesh, measurement_dir / f"{numbers[0]}.yaml",
                     update=True, stdout=False, write_measurements=debug_mode)


if __name__ == '__main__':
    main()
