# Steps that need a maintainer login

These cannot be automated from the repo because they need your own accounts. Nothing here involves sharing a token with anyone.

## 1. Zenodo DOI for the v1.0 release (do this BEFORE publishing the draft release)

1. Sign in at <https://zenodo.org> with **Log in with GitHub** (an account that is an owner/admin of `sparolab`).
2. Open <https://zenodo.org/account/settings/github/>. If `sparolab` repositories are not listed, click **Sync now**; if the org is missing, grant Zenodo access to `sparolab` on GitHub (Settings → Applications → Authorized OAuth Apps → Zenodo → Organization access → Grant).
3. Flip the switch next to **sparolab/SKiD-SLAM** to **On**.
4. On GitHub, open the draft release **v1.0** (Releases → Drafts), check the notes and click **Publish release**. Zenodo archives the tagged source and mints a DOI within a few minutes (it appears under <https://zenodo.org/account/settings/github/repository/sparolab/SKiD-SLAM>). Zenodo reads `CITATION.cff` for the title and authors.
5. Copy the **concept DOI** (the one that always points to the latest version) and add the Zenodo badge to the README link row, and the DOI to `CITATION.cff` under `identifiers` (type `doi`, description "Zenodo software archive").

Order matters: a release published while the Zenodo switch is off is not archived; Zenodo only picks up releases published after it is enabled.

## 2. Hugging Face Papers page

The paper page exists automatically at <https://huggingface.co/papers/2610.38141> (if it does not load yet, use **Submit a paper** on <https://huggingface.co/papers> with the arXiv ID; that also needs a login).

1. Sign in at <https://huggingface.co> (create an account whose name matches the arXiv author name, ideally linked to the same email or ORCID).
2. Open <https://huggingface.co/papers/2610.38141>, click your name in the author list and choose **Claim authorship** ("It's me"); after approval the paper shows on your profile. Co-authors can do the same.
3. On the paper page, use **Add GitHub repository** (or edit the paper's links) and enter `https://github.com/sparolab/SKiD-SLAM`; add the project page `https://sparolab.github.io/research/skid_slam/` as the project page link. The repo is also detected automatically because the README links the arXiv abstract.
4. Once the dataset repo exists (section 3), link it from the paper page the same way (**Add dataset**), and mention `arxiv:2610.38141` in the dataset card so HF links it back.

## 3. Field dataset on Hugging Face Datasets

See `docs/dataset/README.md` (dataset card) and `docs/dataset/upload_to_hf.py`. Before uploading, settle the licence and any data-sharing terms with KICT and with whoever granted access to the lava tube (see the open questions at the top of `docs/dataset/README.md`).

```bash
pip install -U huggingface_hub hf_xet
hf auth login                              # your own token, entered in your terminal only
python docs/dataset/upload_to_hf.py --repo <org>/<name> --src /path/to/skid_slam_dataset --private
# check the files and the rendered card on huggingface.co, then make it public in the repo settings
```
