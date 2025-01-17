<p align="center">
 <img width=640px height=200px src="doc/teaser.png" alt="Project logo">
</p>

<h3 align="center">Masonry Shell Structures with Discrete Equivalence Classes</h3>

<div align="center">

  [![Status](https://img.shields.io/badge/status-active-success.svg)]() 
  [![License](https://img.shields.io/badge/license-MIT-blue.svg)](/LICENSE)

</div>

This repo is an implementation of [Masonry Shell Structures with Discrete Equivalence Classes](doc/Paper.pdf) [Chen et al. 2023]. Commercial licensing is available upon request. If you have any problems when using this code, you can contact me any time through 1998rlchen@gmail.com.

If you make use of this repo in your scientific work, please cite our paper. For your convenience,
you can use the following bibtex snippet:

    @article {Chen-2023-TileableShell, 
    author   = {Rulin Chen and Pengyun Qiu and Peng Song and Bailin Deng and Ziqi Wang and Ying He}, 
    title    = {Masonry Shell Structures with Discrete Equivalence Classes}, 
    journal  = {ACM Transactions on Graphics (SIGGRAPH 2023)}, 
    volume   = {42}, 
    number   = {4}, 
    year     = {2023}} 

## Table of Contents
- [About](#about)
- [Getting Started](#getting_started)
- [GUI Interface](#usage)
- [Create a Masonry Shell Structures with Discrete Equivalence Classes!](#create_puzzle)
- [Authors](#authors)
- [Acknowledgments](#acknowledgement)

## About <a name = "about"></a>
This repo presents a computational approach to design masonry shell structures with discrete equivalence classes. We implemented our computational design tool in C++ and `libigl` [Jacobson et al. 2018] on a desktop computer with 3.6 GHz 8-Core Intel processor and 16 GB RAM. 

## Getting Started <a name = "getting_started"></a>
Our code can be ran on MacOS and Unbuntu (Linux) system. First clone the repository, run CMake to generate Makefiles or CMake/Visual Studio project files, and the rest should just work automatically.

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

Please note that the code has been **tested** on Ubuntu environment (Thanks Vincent Nivoliers!). The instructions below are specifically for an Ubuntu-based container.

To compile the code in the Ubuntu environment, the following packages were required:

- `libboost-filesystem-dev`
- `libboost-thread-dev`
- `libgmp-dev`
- `libmpfr-dev`
- `libglfw3-dev`
- `libxinerama-dev`
- `libxi-dev`

After compilation, an additional package was needed to load the remeshed mesh:

- `zenity`

Ensure all these dependencies are installed to avoid any issues during the build and execution phases.

- **Windows**: our code can be run on VM16 + Ubuntu 20.04 environment.


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

  `Avg Contact Angle Error`, `Avg Overlap Ratio` and `Avg Gap Ratio` are three error metrics defined in Section 3 of our paper. These metric values will be updated after creating a shell structure successfully by clicking `Create Shell` button.

- ### Import & Remesh

  Our algorithm mainly supports the polygonal architecture surface with open boundary. Here we provide two types of input modes: 
  
  1. Obtain the base mesh by parameterizing the surface using the as-rigid-as-possible (ARAP) algorithm [Liu et al. 2008] and then mapping a 2D tessellation with convex polygons onto the surface. This process can be done by using the following group of buttons. You can find the input surfaces and tile patterns in */data/Model* and */data/TilePattern*, respectively.

  &emsp;`Scalar`  Specify the scalar of the input tile pattern. Users can use keyboard `M` and `N` to scale up / down the tile pattern interactively.
  
  &emsp;`RotAngle` Specify the rotation angle of the input tile pattern. Users can use keyboard `U` and `I` to rotate the tile pattern interactively.
  
  &emsp;`Read` Load the input surface.
  
  &emsp;`Texture` Load the input tile pattern.

  &emsp;`Remesh` Map the 2D tile pattern to input surface using ARAP initialization.

  2. We can directly load the remeshed input surface using `Load Remeshed Mesh`. You can find the example remeshed polygonal surfaces in */data/Model_remeshed*.

  &emsp;`Load Remeshed Mesh` Load the remeshed input mesh.

- ### Mesh Optimization

  `Edge K` Specify the target cluster number of edge length. 

  `Dihedral K` Specify the target cluster number of dihedral angle.

  `Mini Block Cluster` Specify the threshold to define if the block cluster is considered to be merged.

  `Target Block Cluster` Specify the threshold to stop the block cluster merging operation. 

  `Thickness` Specify the thickness of each shell block.

  `Opt Surface` Optimize surface to reach lower number of polygon templates.

  `Opt Cutting Plane` Optimize cutting plane to reach lower number of shell block templates.

  `Create Shell` Create shell structure based on given thickness. 

  `Save Shell` Save optimzied shell structure data. 

- ### Render Control
  Control the object visualization state.

## Create a Masonry Shell Structures with Discrete Equivalence Classes by Yourself ! <a name = "create_puzzle"></a>
These instructions gives an example to you of how to use our code to generate a masonry shell structures with discrete equivalence classes. 

### Step 1: import input files
Import a Hyperbolic.obj file and 4^4_7200.obj by clicking `read` and `texture` button, respectively. Here we support two kinds of input mode: `read` and `texture` for remeshing models here OR `Load Remeshed Mesh` for load your own remeshed models. Set `Scalar` as 5.0 and `RotAngle` as -15. Then click the `remesh` button to get a remeshed surface.

<p align="center">
 <img width=640px height=400px src="doc/Step1.png" alt="Remeshed Surface">
</p>

### Step 2: optimize suface
Set the `Edge K` as 2 and `Dihedral K` as 4, click the `Opt Surface` button to optimize surface first. 

<p align="center">
 <img width=640px height=400px src="doc/Step2.png" alt="Optimized Surface">
</p>

### Step 3: optimize cutting plane
Set the `Mini Block Cluster` as 10 and `Target Block Cluster` as 1. Set `Thickness` as 0.08. Click the `Opt CuttingPlane` button to start cutting plane optimization. Then click the `Create Shell` button to create shell structures with given thickness.

<p align="center">
 <img width=640px height=400px src="doc/Step3.png" alt="Optimized Shell Structure">
</p>

### Step 4: save optimized model
Lastly, you can click `Save Shell` to save the related data and *.obj* files that can be used for fabrication. Here we suggest to use MeshLab to view the generated *.obj* files.

<p align="center">
 <img width=540px height=160px src="doc/Step4.png" alt="Saving Files">
</p>

## Authors <a name = "authors"></a>
- [Rulin Chen](https://linsanity81.github.io) 
- [Pengyun Qiu](https://sutd-cgl.github.io/people.html)
- [Peng Song](https://songpenghit.github.io/)
- [Bailin Deng](http://www.bdeng.me)
- [Ziqi Wang](https://kiki007.github.io/)
- [Ying He](https://personal.ntu.edu.sg/yhe/)

## Acknowledgements <a name = "acknowledgement"></a>
We thank the reviewers for their valuable comments, and Zebin Chen for helping on preparing some 3D surface models and renderings. This work was supported by the SUTD grant RS-INSUR-00027, the Ministry of Education, Singapore, under its Academic Research Fund Grants (RG20/20 & MOE-T2EP20220-0014), and the Royal Society (IES\R3\193208).











