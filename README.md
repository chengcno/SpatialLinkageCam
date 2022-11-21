# 3D CamLinkages Program

[![Status](https://img.shields.io/badge/status-active-success.svg)]()
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](/LICENSE)

![](/Users/yingjie/Documents/spatialLink_backup/doc/teaser.png) 

This repo is an implementation of [Exact 3D Path Generation via 3D Cam-Linkage Mechanisms](https://sutd-cgl.github.io/supp/Publication/projects/2022-SIGAsia-3DCamLinkage/index.html) [Cheng et al. 2022].
If you have any problems when using this code, you can contact me any time through chengyj@mail.ustc.edu.cn

If you make use of this repo in your scientific work, please cite our paper. For your convenience,
you can use the following bibtex snippet:

    @article {Cheng-2022-3DCamLinkage,
    author  = {Yingjie Cheng and Peng Song and Yukun Lu and Wen Jie Jeremy Chew and Ligang Liu},
    title   = {Exact 3D Path Generation via 3D Cam-Linkage Mechanisms},
    journal = {ACM Transactions on Graphics (SIGGRAPH Asia 2022)},
    volume  = {41},
    number  = {6},
    pages   = {225:1 -- 225:13},
    year    = {2022}}

## Table of Contents
- [About](#about)
- [Getting Started](#getting_started)
- [GUI Interface](#usage)
- [Show a Demo](#show_demo)
- [Create a Cam-Linkage Mechanism](#create_mech)
- [Acknowledgments](#acknowledgement)

## About <a name = "about"></a>
This repo presents a computational approach of designing a cam-linkage mechanism to make a point exactly move along a prescribed 3D path, driven by a single actuator.
We implemented our computational design tool in C++ and `libigl` [Jacobson et al. 2018] on a desktop computer with 3.6 GHz 8-Core Intel processor and 16 GB RAM.

## Getting Started <a name = "getting_started"></a>
Our code can be ran on MacOS and Unbuntu (Linux) system. First clone the repository, run CMake to generate Makefiles or CMake/Visual Studio project files, and the rest should just work automatically.


### Compilation

- **MacOS and Ubuntu(Linux)**:

```
cd [current folder path]
mkdir build
cd build
cmake ..
make
```
It's better to build it with `release` because of CGAL.
This should find and build the dependencies and create a `SpatialLinkages_rMain` binary.

- **Windows**: currently unavailable.


## GUI Interface <a name = "usage"></a>
The control panel is shown below. There are 4 components in the control panel: 
**Optimization Process**, **I/O files**, **Operation Control**, **Render Control**.
![](/Users/yingjie/Documents/spatialLink_backup/doc/UI.png)

- ### Parameter Control

  `Read Input Curve`  Read an input curve from folder `/data/InputCurves`

  `Optimization` Optimize a mechanism to realize the input curve.

- ### I/O files

  `Read CamLinkage` Our program can read `mats.dat` files from folder `/data/Demo`

- ### Operation Control

  `Stop Motion`/`Restart Motion` Interruption or continuation of movement

  `Motion Speed` Adjust the motion velocity

- ### Render Control
  Control the object visualization state.

## Show the demo <a name = "show_demo"></a>
You can show the demo by clicking `Read Camlinkage`, importing any `mats.dat` file from the folder `/data/Demo`.
Following is an example about `tennisCurve`:
![](/Users/yingjie/Documents/spatialLink_backup/doc/tennis.png)

## Create a cam-linkage mechanism <a name = "create_mech"></a>
These instructions give an example to you of how to use our code to generate a cam-linkage mechanism by yourself.

### Step 1: import an input curve
Import any input curve file by clicking `Read Input Curve` button.
![](/Users/yingjie/Documents/spatialLink_backup/doc/inputCurve.png)

### Step 2: optimize a mechanism 
Click the `Optimization` button. It would cost about an hour! Break for a cup of coffee.
![](/Users/yingjie/Documents/spatialLink_backup/doc/optModel.png)

### Step 3: control the movement
Use `Restart Motion`,`Stop Motion`,`Motion Speed` to control the movement. Use mouse to adjust the camera view.
![](/Users/yingjie/Documents/spatialLink_backup/doc/motion.png)

## Acknowledgements <a name = "acknowledgement"></a>
We thank the reviewers for their valuable comments, Ke Chen for proofreading the paper, and Robert Ferréol for providing equations of some input parametric curves.
This work was supported by the SUTD Start-up Research Grant (Number: SRG ISTD 2019 148), and the National Natural Science Foundation of China (62025207)
