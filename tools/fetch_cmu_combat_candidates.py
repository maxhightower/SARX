#!/usr/bin/env python3
import re
import shutil
import sys
from pathlib import Path

from huggingface_hub import HfApi, hf_hub_download

REPO_ID = "gbionics/cmu-fbx"

# Deliberately an audition set, not production content. The selected authored
# elbow strike will be promoted separately after Quaternius visual review.
CANDIDATES = [
    (2, 5, "CMU_PunchStrike"),
    (135, 2, "CMU_Empi_A"),
    (135, 3, "CMU_Empi_B"),
    (144, 13, "CMU_LeftPunchSequence"),
    (144, 20, "CMU_PunchSequence"),
]

def resolve(files, subject: int, trial: int) -> str:
    pattern = re.compile(
        rf"(?:^|/)0*{subject}_0*{trial}\.fbx$",
        re.IGNORECASE,
    )
    matches = [path for path in files if pattern.search(path)]
    if len(matches) != 1:
        raise RuntimeError(
            f"expected exactly one FBX for CMU {subject}_{trial}, found {matches}"
        )
    return matches[0]

def main() -> int:
    if len(sys.argv) != 2:
        print("usage: fetch_cmu_combat_candidates.py OUTPUT_DIR", file=sys.stderr)
        return 2

    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)

    api = HfApi()
    files = api.list_repo_files(REPO_ID, repo_type="dataset")

    manifest = []

    for subject, trial, semantic in CANDIDATES:
        repo_path = resolve(files, subject, trial)

        cached = hf_hub_download(
            repo_id=REPO_ID,
            repo_type="dataset",
            filename=repo_path,
        )

        source_id = f"{subject}_{trial:02d}"
        destination = out / f"{source_id}.fbx"

        shutil.copyfile(
            cached,
            destination,
        )

        manifest.append(
            (source_id, semantic, repo_path, destination.stat().st_size)
        )

        print(
            f"{semantic}: source={repo_path} "
            f"local={destination} size={destination.stat().st_size}"
        )

    manifest_path = out / "manifest.tsv"
    with manifest_path.open("w", encoding="utf-8") as handle:
        handle.write("source_id\tsemantic\trepo_path\tsize_bytes\n")
        for row in manifest:
            handle.write("\t".join(map(str, row)) + "\n")

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
