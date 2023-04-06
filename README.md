<p align="center">
 <img width=640px height=260px src="doc/teaser.png" alt="Project logo">
</p>

<h3 align="center">Computational Design of High-level Interlocking Puzzles</h3>

<div align="center">

  [![Status](https://img.shields.io/badge/status-active-success.svg)]() 
  [![License](https://img.shields.io/badge/license-MIT-blue.svg)](/LICENSE)

</div>

This repo is an implementation of [Masonry Shell Structures with Discrete Equivalence Classes](doc/Paper.pdf) [Chen et al. 2023]. Commercial licensing is available upon request. If you have any problems when using this code, you can contact me any time through rulin_chen@mymail.sutd.edu.sg.

If you make use of this repo in your scientific work, please cite our paper. For your convenience,
you can use the following bibtex snippet:

    @article {Chen-2023-MasonryShell,
    author   = {Rulin Chen and Pengyun Qiu and Peng Song and Bailin Deng and  Ziqi Wang and Ying He},
    title        = {Masonry Shell Structures with Discrete Equivalence Classes},
    journal   = {ACM Transactions on Graphics (SIGGRAPH 2023)},
    year      = {2023}}

## Table of Contents
- [About](#about)
- [Getting Started](#getting_started)
- [GUI Interface](#usage)
- [Create a level-12 Cube Puzzle by Yourself !](#create_puzzle)
- [Authors](#authors)
- [Acknowledgments](#acknowledgement)

## About <a name = "about"></a>
This repo presents a computational approach to design high-level interlocking puzzles. We implemented our computational design tool in C++ and `libigl` [Jacobson et al. 2018] on a desktop computer with 3.6 GHz 8-Core Intel processor and 16 GB RAM. 

## Getting Started <a name = "getting_started"></a>
Our code can be ran on MacOS and Unbuntu (Linux) system. First clone the repository, run CMake to generate Makefiles or CMake/Visual Studio project files, and the rest should just work automatically.

<!-- ### Prerequisites
We need to install `cgal` before running our code. With the help of `brew`, we can easily get `cgal`.

```
brew install cgal
``` -->

### Compilation

- **MacOS and Ubuntu(Linux)**:

```
$ cd [current folder path]
$ mkdir build
$ cd build
$ cmake ..
$ make -j 16
```
The integer following make -j is the number of threads supported by your CPU architecture. Replace it with your optimal value.

- **Windows**: currently unavailable.


## GUI Interface <a name = "usage"></a>
The control panel is shown below. There are 4 components in the control panel: **Status Bar**, **Import & Remesh**, **Mesh Optimization** and **Render Control**.
<p align="center">
 <img width=640px height=400px src="doc/GUI.png" alt="Control Panel">
</p>

- ### Status Bar

  `Model` Display the imported surface name.

  `Tile` Display the imported tile pattern name.

  `Tile Number` Display the number of tiles after remeshing.

  `Unique Polygon` Display the number of polygon templates.

  `Unique Block` Display the number of block templates.

- ### Import & Remesh

  `Scalar`  Determine the scalar of the input tile pattern.
  
  `RotAngle` Determine the rotation angle of the input tile pattern. 
  
  `Read` Load the input surface.
  
  `Texture` Load the input tile pattern.

  `Remesh` Map the 2D tile pattern to input surface using ARAP initialization.

- ### Mesh Optimization

  `Edge K` Determine the target cluster number of edge length. 

  `Dihedral K` Determine the target cluster number of dihedral angle.

  `Mini Block Cluster Num` Determine the threshold to define if the block cluster is considered to be merged.

  `Target Block Cluster Num` Determine the threshold to stop the block cluster merging operation. 

  `Thinkness` Determine the thickness of each shell block.

  `Op Surface` Optimize surface to reach lower number of polygon templates.

  `Save Surface` Save optimzied surface data. 

  `Op Cutting Plane` Optimize cutting plane to reach lower number of shell block templates.

  `Save Shell` Save optimzied shell structure data. 

- ### Render Control
  Control the object visualization state.

## Create a level-12 Cube Puzzle by Yourself ! <a name = "create_puzzle"></a>
These instructions gives an example to you of how to use our code to generate a masonry shell structures with discrete equivalence classes. For example, to generate a level-12 cube puzzle, we first constrct a level-6 cube puzzle and modify it to level-12 puzzle; see *Section 4* in our paper for detailed explanation of construction and modifying algorithm.

### Step 1: import & remesh
Import a Flower.obj file and 3^6-6^2_11844.obj by clicking `read` and `texture` button, respectively. Then click the `remesh` button to get a remeshed surface.

<p align="center">
 <img width=640px height=400px src="doc/remeshed_surface.png" alt="Remeshed Surface">
</p>

### Step 2: optimize suface
Set the `Level of Difficulty` as 6 and click the `Construct` button to create a piece-4 level-6 cube puzzle.

<p align="center">
 <img width=640px height=400px src="doc/level-6_puzzle.png" alt="level-6 Puzzle">
</p>

### Step 3: optimize cutting plane
Set the `Target Level (Modify)` as 12 and click the `Modify` button to start modifying.

<p align="center">
 <img width=640px height=400px src="doc/level-12_puzzle.png" alt="Level-12 Puzzle">
</p>

### Step 4: save optimized model
Set the `Puzzle Tolerance` greater than 0 to create gaps between pieces. Here we suggest to set 0.005. Lastly, you can click `Save Puz` to save the *.puz* file and *.obj* files of each piece that can be used for fabrication.

## Authors <a name = "authors"></a>
- [Rulin Chen](https://github.com/Linsanity81) 
- [Pengyun Qiu](https://sutd-cgl.github.io/people.html)
- [Peng Song](https://songpenghit.github.io/)
- [Bailin Deng](http://www.bdeng.me)
- [Ziqi Wang](https://kiki007.github.io/)
- [Ying He](https://personal.ntu.edu.sg/yhe/)

## Acknowledgements <a name = "acknowledgement"></a>
We thank the reviewers for the valuable comments, David Gontier for sharing the source code of the baseline design approach, and Christian Hafner for proofreading the paper. This work was supported by the SUTD start-up Research Grant (Number: SRG ISTD 2019 148), and the European Research Council (ERC) under the European Union's Horizon 2020 research and innovation programme (grant agreement No 715767 - MATERIALIZABLE).











