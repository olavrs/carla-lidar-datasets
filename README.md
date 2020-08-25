# carla-lidar-datasets
This repo contains noiseless point clouds collected using CARLA and a custom solid-state lidar sensor model.

To view the point cloud run the ``view_pclouds.py`` script.

The noise model introduced in the paper is also included in the script.

All datasets are collected using lidars with 200 meter range. The individual datasets are described in more detail in the table below.

| Sim | Environment | Number of lidars | Angular resolutions | Number of points |
|-----|-------------|:----------------:|:-------------------:|:----------------:|
| 0 | Under a bridge. Multiple walls, corners, pillars and cars. | 4 | 0.3° | 485 478 |
| 1 | Empty 4-way crossing.                                      | 4 | 0.3° | 385 199 |
| 2 | Neighborhood. Lots of houses.                              | 4 | 0.3° | 413 423 |
| 3 | Tunnel, during a turn.                                     | 4 | 0.3° | 539 546 |
| 4 | Mountain road in a turn. Trees and stones.                 | 4 | 0.3° | 459 508 |
| 5 | Forest road.                                               | 4 | 0.3° | 487 255 |
| 6 | Under a bridge. Multiple walls, corners, pillars and cars. | 4 | 0.1° | 4 367 078 |
| 7 | Under a bridge. Multiple walls, corners, pillars and cars. | 4 | 0.5° | 174 851 |
| 8 | Under a bridge. Multiple walls, corners, pillars and cars. | 2 | 0.3° | 244 994 |
| 9 | Under a bridge. Multiple walls, corners, pillars and cars. | 6 | 0.3° | 734 012 |
| 10 | Mountain road in a turn. Trees and stones.                | 4 | 0.1° | 4 110 993 |
| 11 | Mountain road in a turn. Trees and stones.                | 4 | 0.5° | 166 055 |
| 12 | Mountain road in a turn. Trees and stones.                | 2 | 0.3° | 239 552 |
| 13 | Mountain road in a turn. Trees and stones.                | 6 | 0.3° | 693 799 |
