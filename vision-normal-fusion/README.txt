
# Compiling ###################################################################

- tested under ubuntu 16.04
- requires cmake, boost and OpenCV
- you can use an IDE 
such as QtCreator to make your life easier


Run the following commands:

cd vision-normal-fusion
mkdir build
cd build
cmake ..
make -j4


# Using the code ##############################################################

- 
Example for the cube dataset "datasets/cube"
- "depthMaps" contains the depth maps and normals (raw and visualization)
- use --help to get some information about the 
arugments

To integrate the depth maps run the following commands (from "datasets/cube"):


ls depthMaps/depth*.dat > depthMaps.txt
../../build/tvHistDepthIntegrator --configFile conf.txt --depthMapListFile depthMaps.txt --outputGridFile intCSDFGrid.dat

-------
------------------------------------------------------------------------

To run the TV-Hist algorithm run the following command:


../../build/tvHistFusion --configFile conf.txt --intDepthGridFile intCSDFGrid.dat --vrmlOutputFile tvHistModel.wrl

open the tvHistModel.wrl file using MeshLab to see 
the result.

-------------------------------------------------------------------------------


To integrate the normals run the following commands:

ls depthMaps/normals*.dat > normals.txt
../../build/tvHistNormalIntegrator --configFile conf.txt --depthMapListFile depthMaps.txt --normalMapListFile normals.txt 
--outputFolder integratedNormals

-------------------------------------------------------------------------------


To integrate the normals run the following commands:


ls integratedNormals/normals*.dat > normalsFiles.txt
../../build/tvHistNormalClustering --configFile conf.txt --normalsListFile normalsFiles.txt 
--outputFile clusteredNormals.dat

-------------------------------------------------------------------------------


To run a TV-Hist algorithm that takes the normals into account run the command:

../../build/tvHistNormalFusion --configFile conf.txt --intDepthGridFile intCSDFGrid.dat
 --clusteredNormalsFile clusteredNormals.dat --vrmlOutputFile tvHistNormalClusteringModel.wrl

--------------------------------------------------------------------------
-----

