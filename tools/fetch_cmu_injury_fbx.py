#!/usr/bin/env python3
import os
import shutil
import sys
from pathlib import Path

from huggingface_hub import HfApi, hf_hub_download

REPO_ID = "gbionics/cmu-fbx"
CLIPS = {
    "91_16.fbx": "CMU_Limp",
    "91_24.fbx": "CMU_HurtLegWalk",
    "91_25.fbx": "CMU_DragBadLegWalk",
    "139_19.fbx": "CMU_WalkWoundedLeg",
    "142_12.fbx": "CMU_PainfulLeftKnee",
}

def main() -> int:
    if len(sys.argv) != 2:
        print("usage: fetch_cmu_injury_fbx.py OUTPUT_DIR", file=sys.stderr)
        return 2

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)

    api = HfApi()
    files = api.list_repo_files(REPO_ID, repo_type="dataset")

    for filename, sarx_name in CLIPS.items():
        matches = [path for path in files if path.endswith("/" + filename) or path == filename]
        if len(matches) != 1:
            raise RuntimeError(
                f"expected exactly one repository path for {filename}, found {matches}"
            )

        repo_path = matches[0]
        print(f"{sarx_name}: {repo_path}")
        cached = hf_hub_download(
            repo_id=REPO_ID,
            repo_type="dataset",
            filename=repo_path,
        )
        destination = out / filename
        shutil.copyfile(cached, destination)
        print(f"downloaded {destination} size={destination.stat().st_size}")

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
