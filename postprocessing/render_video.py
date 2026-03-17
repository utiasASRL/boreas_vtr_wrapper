import argparse
import cv2
import numpy as np
from pathlib import Path

def create_depth_video(input_dir, output_file, fps=30):
    in_path = Path(input_dir)
    out_path = Path(output_file)

    out_path.parent.mkdir(parents=True, exist_ok=True)

    pngs = list(in_path.glob("*.png"))
    pngs = sorted(pngs, key=lambda p: int(p.stem))  

    if not pngs:
        raise RuntimeError(f"No depth images found in {in_path}!")

    first_img = cv2.imread(str(pngs[0]), cv2.IMREAD_UNCHANGED)
    H0, W0 = first_img.shape[:2]

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, fps, (W0, H0), isColor=True)

    for p in pngs:
        # IMREAD_UNCHANGED keeps 16-bit as 16-bit, and keeps color channels intact
        img = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)

        # 1. Handle precision depth (scale 16-bit down to 8-bit)
        if img.dtype == np.uint16:
            vis = (img.astype(np.float32) / 65535.0 * 255.0).astype(np.uint8)
        else:
            vis = img.astype(np.uint8)

        # 2. Handle Color Channels dynamically
        if len(vis.shape) == 2:
            # It's a 1-channel grayscale image (H, W) -> Convert to 3-channel
            vis_bgr = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
            
        elif len(vis.shape) == 3 and vis.shape[2] == 3:
            # It's already a 3-channel BGR colormap (H, W, 3) -> Do nothing
            vis_bgr = vis
            
        elif len(vis.shape) == 3 and vis.shape[2] == 4:
            # Catch-all: If it somehow has an Alpha channel (BGRA), strip it
            vis_bgr = cv2.cvtColor(vis, cv2.COLOR_BGRA2BGR)
            
        else:
            print(f"Warning: Skipping {p.name} due to unexpected shape {vis.shape}")
            continue

        writer.write(vis_bgr)

    writer.release()
    print(f"Successfully wrote depth video to: {out_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate an MP4 video from a folder of depth PNGs.")
    parser.add_argument("-i", "--input", type=str, required=True, 
                        help="Path to the directory containing the PNG frames")
    parser.add_argument("-o", "--output", type=str, required=True, 
                        help="Path and filename for the output video (e.g., /path/to/video.mp4)")
    parser.add_argument("--fps", type=int, default=30, 
                        help="Frames per second for the output video (default: 30)")

    args = parser.parse_args()
    create_depth_video(args.input, args.output, args.fps)