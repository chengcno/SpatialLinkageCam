# 3D CamLinkages Program

A project about 3D CamLinkages.
1) Read a input curve, optimize and design a 3D cam-linkage to realize it.
2) Read and show the motion of optimal results.

## Dependencies

The only dependencies are stl, eigen, pugixml, [libigl](http://libigl.github.io/libigl/), CGAL and
the dependencies of the `igl::opengl::glfw::Viewer`.

 libigl and pugixml are added to this git repo as submodule

## Compile

It's better to build it with `release` because of CGAL.

Compile this project using the standard cmake routine:

    mkdir build
    cd build
    cmake ..
    make

This should find and build the dependencies and create a `SpatialLinkages_rMain` binary.

## Run
You would see a cross ground and 3 axes.

## Operation

1) Click 'Read Input Curve', choose one curve from folder '0_InputCurves'
2) Click 'Optimization', this program will design a 3D cam-linkage. It may take about one hour.

3) Click 'Read Cam Linkage', choose one in folder '0_Demo', and open any 'mats.dat'.
4) Use 'Stop/Restart motion' to control animation. Use "Animation Speed" to adjust motion velocity.
5) 'Render control' can show/hide models, ground, axes.


