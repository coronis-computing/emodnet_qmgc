# Documentation

The documentation has two parts, published together on
[GitHub Pages](https://coronis-computing.github.io/emodnet_qmgc/):

| Part | Source | Tool | Published at |
|------|--------|------|--------------|
| User and developer guide (this site) | `docs/*.md`, `mkdocs.yml` | [MkDocs](https://www.mkdocs.org/) + [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) | `/` |
| C++ API reference | Doxygen comments in `src/`, `doxyfile.cfg` | [Doxygen](https://www.doxygen.nl/) | `/api/` |

The **Changelog** and **Implementation notes** pages include `Changelog.md` and `DeveloperNotes.md` from the root of
the repository, so edit those files directly.

## Writing and previewing locally

Install the Python requirements (a virtual environment is recommended):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-docs.txt
```

Start the live-reloading preview server, then open <http://127.0.0.1:8000>:

```bash
mkdocs serve
```

Build the static site into `site/`, failing on any warning (e.g., broken links), as the CI does:

```bash
mkdocs build --strict
```

### Files

```text
mkdocs.yml                     # site configuration and navigation (nav)
requirements-docs.txt          # Python packages needed to build the site
docs/
├── index.md                   # home page
├── getting-started/           # installation, quick start, tutorial
├── user-guide/                # qm_tiler, TIN creation, Cesium, scripts, dem2tin
├── developer/                 # this section
├── assets/viewer.html         # example Cesium viewer (downloadable, included in pages)
├── logos/                     # Coronis and EMODnet logos (also used by Doxygen)
├── stylesheets/extra.css      # colors, footer and logo styles
└── javascripts/mathjax.js     # MathJax setup for formulas
overrides/partials/copyright.html  # footer with the logos
```

To add a page, create the Markdown file under `docs/` and add it to the `nav` section of `mkdocs.yml`.

### Useful syntax

Material for MkDocs supports, among
[others](https://squidfunk.github.io/mkdocs-material/reference/):

- admonitions: `!!! note`, `!!! warning`, `!!! tip`;
- formulas: `\( p_z = p_0 / 2^z \)` inline, `\[ ... \]` as a block;
- diagrams: fenced code blocks with the `mermaid` language;
- including files: `--8<-- "path/from/repo/root"`.

## Automatic publication with GitHub Actions

The workflow `.github/workflows/docs.yml` builds and publishes the documentation:

- **On every push to `master`** that changes the documentation, the sources or the Doxygen configuration, it builds
  both parts, and deploys them to GitHub Pages.
- **On pull requests** and **pushes to `devel`**, it only builds them, with `mkdocs build --strict`, so broken links
  or a broken configuration are caught before merging. Nothing is deployed.
- It can also be started by hand from the *Actions* tab (*Run workflow*). Started on `master`, this run deploys too.

The workflow deploys through the official GitHub Pages actions (`actions/upload-pages-artifact` and
`actions/deploy-pages`), so it needs no access token or `gh-pages` branch.

### One-time setup

In the GitHub repository, go to **Settings → Pages**, and under **Build and deployment → Source**, select
**GitHub Actions**.

That is all. The site is then published at `https://coronis-computing.github.io/emodnet_qmgc/` after the next push to
`master`.

!!! note "The old `gh-pages` branch"
    The Doxygen documentation used to be published from the `gh-pages` branch. Once the Pages source is set to *GitHub
    Actions*, that branch is no longer used, and you can delete it. The workflow keeps the old
    `/html/index.html` address working with a redirect to the API reference.
