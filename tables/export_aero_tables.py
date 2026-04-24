#!/usr/bin/env python3
"""Export F-16 aerodynamic coefficient tables from F16AeroData.h5.

This mirrors the coefficient definitions in preprocess_F16_AeroData.m and
writes one whitespace-delimited text file per coefficient. Each file contains
the independent variables followed by the coefficient value column.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

try:
    import h5py
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    raise SystemExit(
        "This script requires 'h5py'. Install it with 'python3 -m pip install h5py'."
    ) from exc


@dataclass(frozen=True)
class CoefficientSpec:
    name: str
    dataset: str
    grid_names: tuple[str, ...]


COEFFICIENT_SPECS: tuple[CoefficientSpec, ...] = (
    CoefficientSpec("Cx", "_Cx", ("alpha1", "beta1", "dh1")),
    CoefficientSpec("Cy", "_Cy", ("alpha1", "beta1")),
    CoefficientSpec("Cz", "_Cz", ("alpha1", "beta1", "dh1")),
    CoefficientSpec("Cl", "_Cl", ("alpha1", "beta1", "dh2")),
    CoefficientSpec("Cm", "_Cm", ("alpha1", "beta1", "dh1")),
    CoefficientSpec("Cn", "_Cn", ("alpha1", "beta1", "dh2")),
    CoefficientSpec("Cx_lef", "_Cx_lef", ("alpha2", "beta1")),
    CoefficientSpec("Cy_lef", "_Cy_lef", ("alpha2", "beta1")),
    CoefficientSpec("Cz_lef", "_Cz_lef", ("alpha2", "beta1")),
    CoefficientSpec("Cl_lef", "_Cl_lef", ("alpha2", "beta1")),
    CoefficientSpec("Cm_lef", "_Cm_lef", ("alpha2", "beta1")),
    CoefficientSpec("Cn_lef", "_Cn_lef", ("alpha2", "beta1")),
    CoefficientSpec("Cxq", "_Cxq", ("alpha1",)),
    CoefficientSpec("Cyp", "_Cyp", ("alpha1",)),
    CoefficientSpec("Czq", "_Czq", ("alpha1",)),
    CoefficientSpec("Cmq", "_Cmq", ("alpha1",)),
    CoefficientSpec("Cyr", "_Cyr", ("alpha1",)),
    CoefficientSpec("Cnr", "_Cnr", ("alpha1",)),
    CoefficientSpec("Cnp", "_Cnp", ("alpha1",)),
    CoefficientSpec("Clp", "_Clp", ("alpha1",)),
    CoefficientSpec("Clr", "_Clr", ("alpha1",)),
    CoefficientSpec("deltaCxq_lef", "_deltaCxq_lef", ("alpha2",)),
    CoefficientSpec("deltaCyr_lef", "_deltaCyr_lef", ("alpha2",)),
    CoefficientSpec("deltaCyp_lef", "_deltaCyp_lef", ("alpha2",)),
    CoefficientSpec("deltaCzq_lef", "_deltaCzq_lef", ("alpha2",)),
    CoefficientSpec("deltaClr_lef", "_deltaClr_lef", ("alpha2",)),
    CoefficientSpec("deltaClp_lef", "_deltaClp_lef", ("alpha2",)),
    CoefficientSpec("deltaCmq_lef", "_deltaCmq_lef", ("alpha2",)),
    CoefficientSpec("deltaCnr_lef", "_deltaCnr_lef", ("alpha2",)),
    CoefficientSpec("deltaCnp_lef", "_deltaCnp_lef", ("alpha2",)),
    CoefficientSpec("Cy_r30", "_Cy_r30", ("alpha1", "beta1")),
    CoefficientSpec("Cn_r30", "_Cn_r30", ("alpha1", "beta1")),
    CoefficientSpec("Cl_r30", "_Cl_r30", ("alpha1", "beta1")),
    CoefficientSpec("Cy_a20", "_Cy_a20", ("alpha1", "beta1")),
    CoefficientSpec("Cy_a20_lef", "_Cy_a20_lef", ("alpha2", "beta1")),
    CoefficientSpec("Cn_a20", "_Cn_a20", ("alpha1", "beta1")),
    CoefficientSpec("Cn_a20_lef", "_Cn_a20_lef", ("alpha2", "beta1")),
    CoefficientSpec("Cl_a20", "_Cl_a20", ("alpha1", "beta1")),
    CoefficientSpec("Cl_a20_lef", "_Cl_a20_lef", ("alpha2", "beta1")),
    CoefficientSpec("deltaCnbeta", "_deltaCnbeta", ("alpha1",)),
    CoefficientSpec("deltaClbeta", "_deltaClbeta", ("alpha1",)),
    CoefficientSpec("deltaCm", "_deltaCm", ("alpha1",)),
    CoefficientSpec("eta_el", "_eta_el", ("dh1",)),
)


DISPLAY_NAMES = {
    "alpha1": "alpha",
    "alpha2": "alpha",
    "beta1": "beta",
    "dh1": "ele",
    "dh2": "ele",
}


def load_grid(h5_file: h5py.File, grid_name: str) -> np.ndarray:
    return np.asarray(h5_file[grid_name]).reshape(-1)


def matlab_reshape(data: np.ndarray, shape: tuple[int, ...]) -> np.ndarray:
    flat = np.asarray(data).reshape(-1, order="F")
    expected_size = int(np.prod(shape, dtype=np.int64))
    if flat.size != expected_size:
        raise ValueError(
            f"Cannot reshape dataset of size {flat.size} into shape {shape}."
        )
    return flat.reshape(shape, order="F")


def build_table(grids: Iterable[np.ndarray], values: np.ndarray) -> np.ndarray:
    grid_list = [np.asarray(grid).reshape(-1) for grid in grids]
    mesh = np.meshgrid(*grid_list, indexing="ij")
    columns = [axis.reshape(-1, order="F") for axis in mesh]
    coeff_column = values.reshape(-1, order="F")
    return np.column_stack(columns + [coeff_column])


def export_coefficient(
    h5_file: h5py.File, spec: CoefficientSpec, output_dir: Path, fmt: str
) -> Path:
    grids = [load_grid(h5_file, grid_name) for grid_name in spec.grid_names]
    shape = tuple(len(grid) for grid in grids)
    raw_data = np.asarray(h5_file[spec.dataset])
    values = matlab_reshape(raw_data, shape)
    table = build_table(grids, values)

    header_names = [DISPLAY_NAMES[grid_name] for grid_name in spec.grid_names]
    header = " ".join(header_names + [spec.name])
    output_path = output_dir / f"{spec.name}.dat"
    np.savetxt(output_path, table, fmt=fmt, header=header, comments="")
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export aerodynamic coefficient tables from F16AeroData.h5."
    )
    parser.add_argument(
        "h5_path",
        nargs="?",
        default="F16AeroData.h5",
        help="Path to the input HDF5 file. Default: %(default)s",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        default="aero_table_exports",
        help="Directory for exported table files. Default: %(default)s",
    )
    parser.add_argument(
        "--fmt",
        default="%.10g",
        help="Numeric format passed to numpy.savetxt. Default: %(default)s",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    h5_path = Path(args.h5_path)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    with h5py.File(h5_path, "r") as h5_file:
        exported_paths = [
            export_coefficient(h5_file, spec, output_dir, args.fmt)
            for spec in COEFFICIENT_SPECS
        ]

    print(f"Exported {len(exported_paths)} coefficient tables to {output_dir}")
    for path in exported_paths:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
