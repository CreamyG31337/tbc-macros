# Tests

These run the macros' own filter and geometry code in regular Python 3 (no TBC): `macrocode.py`
loads a macro's constants and its pure-Python section, and `lasread.py` reads LAS/LAZ with the
`las2las` that TBC installs (set `TBC_LAS2LAS` to use another). No packages are needed except for
`experiments/`.

Wherever a script takes `MACRO`, give a macro name (`CleanCloud`, `SeparateObjects`,
`BestFitScans`) or a path to a `.py` file, for example to compare a changed copy.

## Synthetic tests

```
python tests/test_synthetic.py
```

- Clean Point Cloud 3D: a floor and a wall with 4 mm noise and 1,500 outliers; the surfaces must be
  kept, the outliers removed, smoothing must reduce the noise and progress must only go forward.
- Separate Objects: a wall with an anchor bar and a hose lying on it; one click and three clicks
  must select most of each with little wall.
- Best-Fit Scans: a corner of three planes shifted by a known amount must be recovered to 1 mm.

## Scripts for real scans

| Script | What it does |
|---|---|
| `eval_clean.py MACRO LAZ[@inject] ...` | Runs Clean Point Cloud 3D with automatic settings as the macro does and reports what each test removes. `@inject` adds known flyers and clumps first and reports how many are caught. |
| `eval_labelled.py MACRO OUT LABEL=LAZ ...` | Runs the clean on files split by hand (for example `keep=` / `remove=`, or `keep=` / `anchors=` / `junk=`), reports the share of each label removed, and writes a face-on image (`OUT.png`). |
| `simulate_separate.py MACRO wall=LAZ anchors=LAZ junk=LAZ` | Simulates clicking each labelled object with Separate Objects and reports how much of it is selected and how much wall comes with it. |
| `render.py MACRO LAZ OUT --cell C --tol T [--face] [--plan] [--section V U0 U1] [--side U0 U1]` | Images of what the clean removes: face-on, from above, thin cross-sections or side views. |
| `crop.py OUT_DIR minx miny minz maxx maxy maxz NAME=LAZ ...` | Cuts the same box out of several files (las2las), to compare scanners on one section. |

## Experiments

`experiments/` holds the trials that led to Separate Objects' design (see `docs/testing.md`):

- `proud_features.py`: how far each labelled class stands off a trimmed, smoothed rock surface.
- `grow_eval.py`: growing an object from a click through points off a coarse rock surface.
- `local_select_eval.py`: the local-background selection the macro uses, before the connectivity step.
- `objfeat.py`, `make_features.py`, `train_eval.py`: per-point shape features and decision tree /
  random forest classifiers, trained on one half of a labelled wall and tested on the other.
  These need numpy and scikit-learn:
  ```
  uv venv .venv-train
  uv pip install --python .venv-train/Scripts/python.exe numpy scikit-learn
  ```
