# Image and package versions (design §8.2)

Recorded by the lead when images change. apt packages come from the ROS 2 Jazzy and Ubuntu 24.04 (noble) archives
at build time; the base image is pinned by digest.

| Item | Version / digest | Recorded |
|---|---|---|
| Base image `ros:jazzy-ros-base` | `sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca` (Ubuntu 24.04.5) | 2026-10-09 |
| Docker Desktop (host) | 29.8.2, Compose v5.5.1, WSL2 backend | 2026-10-09 |
| Mesa (sim image) | 25.2.8-0ubuntu0.24.04.4 (d3d12 driver used for the WSLg GPU route) | 2026-10-09 |
| colcon-core | 0.21.3 | 2026-10-09 |
| python3-pytest | 7.4.4 | 2026-10-09 |

To refresh: `docker compose run --rm dev bash -c "dpkg -l 'ros-jazzy-*' | awk '/^ii/{print \$2, \$3}'"`.
