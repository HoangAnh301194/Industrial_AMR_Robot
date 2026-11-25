#!/usr/bin/env python3
"""Helper script to save the current SLAM /map into maps/warehouse_map."""

import os
import subprocess
import sys


def main():
    map_name = sys.argv[1] if len(sys.argv) > 1 else "warehouse_map"
    workspace_maps_dir = os.path.join(
        os.getcwd(), "src", "factory_delivery_amr_navigation", "maps"
    )

    if os.path.isdir(os.path.join(os.getcwd(), "src", "factory_delivery_amr_navigation")):
        os.makedirs(workspace_maps_dir, exist_ok=True)
        out_prefix = os.path.join(workspace_maps_dir, map_name)
    else:
        out_prefix = os.path.abspath(map_name)

    cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", out_prefix]
    print(f"[save_warehouse_map] Saving /map to {out_prefix}.yaml and {out_prefix}.pgm ...")
    result = subprocess.run(cmd, check=False)
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
