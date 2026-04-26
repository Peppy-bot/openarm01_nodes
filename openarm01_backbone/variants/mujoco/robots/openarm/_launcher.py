#!/usr/bin/env python3
"""Shared SimLauncher for openarm MuJoCo launch scripts."""

import logging
import os
import sys
import time
from pathlib import Path

logger = logging.getLogger(__name__)

_HEADLESS_ENV = "PEPPY_BRIDGE_HEADLESS"
_EXT_ROOT_ENV = "PEPPY_MUJOCO_EXT_ROOT"
_DEFAULT_EXT_ROOT = (
    Path(__file__).resolve().parents[3] / "exts" / "mujoco.peppy.backbone"
)


class SimLauncher:
    """Manages the MuJoCo lifecycle for an openarm robot."""

    def __init__(self, xml_path: Path) -> None:
        self._xml_path = xml_path
        self._headless = os.environ.get(_HEADLESS_ENV, "0").strip() == "1"

    def run(self) -> None:
        """Load model, init bridge, and drive the simulation loop."""
        import mujoco  # pylint: disable=E0401

        if not self._xml_path.exists():
            raise FileNotFoundError(
                f"MJCF not found at {self._xml_path} — "
                "download from https://github.com/enactic/openarm"
                " and place it in assets/"
            )

        logger.info(f"Loading model: {self._xml_path}")
        model = mujoco.MjModel.from_xml_path(str(self._xml_path))
        data = mujoco.MjData(model)

        ext_root = Path(os.environ.get(_EXT_ROOT_ENV, str(_DEFAULT_EXT_ROOT)))
        sys.path.insert(0, str(ext_root))

        from peppy_mujoco.backbone.extension import (
            MujocoBackboneExtension,
        )  # pylint: disable=E0401

        extension = MujocoBackboneExtension(model, data)
        extension.startup()

        logger.info("Simulation running — Press Ctrl-C to stop.")
        logger.info("Publishing sensor topics: MuJoCo → openarm01_backbone")

        if self._headless:
            self._run_headless(model, extension)
        else:
            self._run_windowed(model, data, extension)

    def _run_headless(self, model, extension) -> None:
        """Drive simulation loop at fixed timestep without a viewer."""
        dt = model.opt.timestep
        try:
            while True:
                t0 = time.perf_counter()
                extension.step()
                remaining = dt - (time.perf_counter() - t0)
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            logger.info("Shutting down.")
        finally:
            extension.shutdown()

    def _run_windowed(self, model, data, extension) -> None:
        """Drive simulation loop at fixed timestep with the passive MuJoCo viewer."""
        import mujoco.viewer  # pylint: disable=E0401

        dt = model.opt.timestep
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                while viewer.is_running():
                    t0 = time.perf_counter()
                    extension.step()
                    viewer.sync()
                    remaining = dt - (time.perf_counter() - t0)
                    if remaining > 0:
                        time.sleep(remaining)
        except KeyboardInterrupt:
            logger.info("Shutting down.")
        finally:
            extension.shutdown()
