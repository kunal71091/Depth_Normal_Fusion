This is a modified version of the IsoEx library from the University of Aachen.

It was used to test the EMC algorithm for isosurface extraction after the optimization step of the depth map fusion.

It takes a data file as input where the scalar distance field U from the optimization step is stored.

The application can be found in ./IsoEx/App3/emc_ethz/emc_ethz.cc

The "data.txt" file needs to be located in the same directory as the binary.

For now the grid resolution is still hardcoded in the app, so change this, and recompile, if you want to load other data.


