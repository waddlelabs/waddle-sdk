# Reference scene appearance

`wood_table_001_diff_1k.png` is the unmodified 1K diffuse PNG from
[Poly Haven's Wood Table 001](https://polyhaven.com/a/wood_table_001), distributed
under [CC0](https://polyhaven.com/license). It is a public-domain asset, not an
exclusive SDK asset. The PNG adds 5,293,264 bytes to the package; rendering never
downloads it. Only visual materials use it, without changing collision geometry.

Source: https://dl.polyhaven.org/file/ph-assets/Textures/png/1k/wood_table_001/wood_table_001_diff_1k.png

SHA-256: `aef45ab36d9f258b80edf278d417920b7326ad77c341d6618498d4598c59fd2c`

The shared URDF references this texture. MuJoCo also receives an explicit native
texture/material binding because its URDF importer does not import textures.
SAPIEN uses its native PBR materials; neither renderer needs a ray-tracing mode
for these defaults. Native Isaac appearance still requires licensed acceptance.
