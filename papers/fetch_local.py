#!/usr/bin/env python3
"""Download SARX research PDFs for local study.

Only the explicitly redistributable PDF in vendor-manifest.tsv is intended
for git. The other files below are downloaded to papers/local/ and are
intentionally gitignored.
"""

from pathlib import Path
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "local"

PAPERS = [
    (
        "mcgraw_2024_gram_schmidt_voxel_constraints.pdf",
        "https://web.ics.purdue.edu/~tmcgraw/papers/mcgraw_mig_2024.pdf",
    ),
    (
        "mcgraw_zhou_2025_voxel_mesh_fracture.pdf",
        "https://web.ics.purdue.edu/~tmcgraw/papers/voxel_2025.pdf",
    ),
    (
        "mcgraw_myers_2026_dissectible_deformable_models.pdf",
        "https://dl.acm.org/doi/pdf/10.1145/3820013",
    ),
    (
        "benchekroun_2023_fast_complementary_dynamics.pdf",
        "https://www.dgp.toronto.edu/projects/fast_complementary_dynamics_site/Fast_Complementary_Dynamics_Low_res.pdf",
    ),
    (
        "chang_2025_wind_lifter.pdf",
        "https://www.dgp.toronto.edu/projects/windlifter/assets/windlifter.pdf",
    ),
    (
        "macklin_2016_xpbd.pdf",
        "http://mmacklin.com/xpbd.pdf",
    ),
]

def download(name: str, url: str) -> None:
    destination = OUT / name
    if destination.exists() and destination.stat().st_size > 0:
        print(f"exists: {destination}")
        return

    print(f"fetch:  {url}")
    request = Request(url, headers={"User-Agent": "SARX research paper fetcher/1.0"})
    try:
        with urlopen(request, timeout=120) as response, destination.open("wb") as out:
            while True:
                chunk = response.read(1024 * 1024)
                if not chunk:
                    break
                out.write(chunk)
    except (HTTPError, URLError, TimeoutError) as exc:
        destination.unlink(missing_ok=True)
        print(f"failed: {name}: {exc}")
        return

    print(f"saved:  {destination} ({destination.stat().st_size / 1024 / 1024:.1f} MiB)")

def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    for name, url in PAPERS:
        download(name, url)

if __name__ == "__main__":
    main()
