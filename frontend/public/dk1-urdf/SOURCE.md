# TRLC-DK1 follower URDF and meshes

`TRLC-DK1-Follower.urdf` and `meshes/` are copied verbatim from
[robot-learning-co/trlc-dk1](https://github.com/robot-learning-co/trlc-dk1)
(`urdf/follower/`), Copyright 2025-2026 The Robot Learning Company UG
(haftungsbeschränkt), licensed under the Apache License 2.0 — the same license
as LeLab. They are vendored here because the 3D viewer loads them over HTTP
from `frontend/public/`.

`TRLC-DK1-Follower-bimanual.urdf` is generated from the file above by
[`scripts/make_dk1_bimanual_urdf.py`](../../../scripts/make_dk1_bimanual_urdf.py)
— do not edit it by hand. Regenerate it (and refresh the copies above from
upstream) with:

```bash
python scripts/make_dk1_bimanual_urdf.py
```
