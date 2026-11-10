#!/usr/bin/env python3
"""Upload the SKiD-SLAM field dataset to the Hugging Face Hub.

Run it yourself after `hf auth login` (older huggingface_hub: `huggingface-cli login`) (the token stays in your own ~/.cache/huggingface).
The source folder must follow the layout in docs/dataset/README.md:

    <src>/README.md                         dataset card (copy docs/dataset/README.md and fill it in)
    <src>/majang/{ros1,ros2}/...            bags
    <src>/majang/{gt,calib}/...
    <src>/planetary_analog/{ros1,ros2}/...
    <src>/planetary_analog/{gt,calib}/...

huggingface_hub >= 1.0 (with hf_xet) uploads a large folder in several commits and resumes an
interrupted upload when re-run with the same arguments; older versions use upload_large_folder,
which does the same.

Example:
    python docs/dataset/upload_to_hf.py --repo sparolab/SKiD-SLAM-Field --src /data/skid_slam_dataset --private
    python docs/dataset/upload_to_hf.py --repo sparolab/SKiD-SLAM-Field --src /data/skid_slam_dataset --dry-run
"""
import argparse
import os
import sys

from huggingface_hub import HfApi

ALLOWED = ["README.md", "LICENSE*", "*.bag", "*.db3", "*.mcap", "metadata.yaml", "*.txt", "*.csv",
           "*.yaml", "*.json", "*.pcd", "*.ply", "*.las", "*.laz", "*.png", "*.jpg", "*.mp4"]
IGNORED = ["**/.git/**", "**/*.log", "**/__pycache__/**", "**/*.tmp", "**/.DS_Store"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", required=True, help="<org or user>/<dataset name> on the Hub")
    ap.add_argument("--src", required=True, help="local dataset root (with README.md)")
    ap.add_argument("--private", action="store_true", help="create the repo private (recommended; publish after checking)")
    ap.add_argument("--workers", type=int, default=8, help="parallel workers (huggingface_hub < 1.0 only)")
    ap.add_argument("--dry-run", action="store_true", help="list what would be uploaded and the total size, then stop")
    a = ap.parse_args()

    if not os.path.isfile(os.path.join(a.src, "README.md")):
        sys.exit(f"{a.src}/README.md (the dataset card) is missing")

    total, files = 0, []
    for root, _, names in os.walk(a.src):
        for n in names:
            p = os.path.join(root, n)
            if n.endswith(".log") or "/.git/" in p:
                continue
            files.append(os.path.relpath(p, a.src))
            total += os.path.getsize(p)
    print(f"{len(files)} files, {total / 1e9:.1f} GB under {a.src}")
    if a.dry_run:
        for f in sorted(files):
            print("  ", f)
        return

    api = HfApi()
    who = api.whoami()["name"]
    print(f"logged in as {who}; uploading to datasets/{a.repo} ({'private' if a.private else 'public'})")
    api.create_repo(a.repo, repo_type="dataset", private=a.private, exist_ok=True)
    if hasattr(api, "upload_large_folder"):          # huggingface_hub < 1.0
        api.upload_large_folder(repo_id=a.repo, repo_type="dataset", folder_path=a.src,
                                allow_patterns=ALLOWED, ignore_patterns=IGNORED, num_workers=a.workers)
    else:                                            # >= 1.0: batched, resumable through hf_xet
        api.upload_folder(repo_id=a.repo, repo_type="dataset", folder_path=a.src,
                          allow_patterns=ALLOWED, ignore_patterns=IGNORED,
                          commit_message="Upload SKiD-SLAM field dataset")
    print(f"done: https://huggingface.co/datasets/{a.repo}")


if __name__ == "__main__":
    main()
