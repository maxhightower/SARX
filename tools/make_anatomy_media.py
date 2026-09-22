#!/usr/bin/env python3
"""Render the V0.5 anatomy demos and encode MP4 + GIF previews.

Usage: python tools/make_anatomy_media.py --demo build/sarx_anatomy_demo [--out media]
Requires: pillow, imageio-ffmpeg (pip install pillow imageio-ffmpeg).
"""
import argparse, glob, os, subprocess, tempfile

import imageio_ffmpeg
from PIL import Image

SCENARIOS = {
    "hack": "v05_anatomy_hack_through_torso",
    "limb": "v05_anatomy_two_chop_arm_severance",
    "shoot": "v05_anatomy_ballistics",
    "rip": "v05_anatomy_tear_in_half",
}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", required=True)
    ap.add_argument("--out", default="media")
    ap.add_argument("--scenario", default="all")
    args = ap.parse_args()
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    with tempfile.TemporaryDirectory() as tmp:
        for sc, stem in SCENARIOS.items():
            if args.scenario not in ("all", sc):
                continue
            subprocess.run([args.demo, "--scenario", sc, "--output", tmp], check=True)
            frames = sorted(glob.glob(os.path.join(tmp, sc, "frame_*.ppm")))
            pngdir = os.path.join(tmp, sc + "_png")
            os.makedirs(pngdir, exist_ok=True)
            for i, f in enumerate(frames):
                Image.open(f).save(os.path.join(pngdir, "f%04d.png" % i))
            mp4 = os.path.join(args.out, "mp4", stem + ".mp4")
            gif = os.path.join(args.out, "gif", stem + ".gif")
            os.makedirs(os.path.dirname(mp4), exist_ok=True)
            os.makedirs(os.path.dirname(gif), exist_ok=True)
            subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-framerate", "60", "-i",
                            os.path.join(pngdir, "f%04d.png"), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                            "-crf", "23", mp4], check=True)
            subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-i", mp4, "-vf",
                            "fps=20,scale=480:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse",
                            gif], check=True)
            print(sc, "->", mp4, os.path.getsize(mp4) // 1024, "KB;", gif, os.path.getsize(gif) // 1024, "KB")

if __name__ == "__main__":
    main()
