#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <iterator>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include <IsoEx/Implicits/ImplicitSphere.hh>
#include <IsoEx/Implicits/csg.hh>

#include <IsoEx/Grids/ScalarGridT.hh>

#include <IsoEx/Extractors/MarchingCubesT.hh>
#include <IsoEx/Extractors/ExtendedMarchingCubesT.hh>

#define VectorType OpenMesh::Vec3f

//-----------------------------------------------------------------------------


using namespace IsoEx;
using namespace OpenMesh;
using namespace OpenMesh::IO;


template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

//-----------------------------------------------------------------------------


// Define the mesh to be used: need vertex normal and status for EMC
struct MyTraits : public DefaultTraits
{
  VertexAttributes   (Attributes::Normal | Attributes::Status);
  HalfedgeAttributes (Attributes::PrevHalfedge);

  /// Use double precision points
  typedef VectorType Point;
  /// Use double precision Normals
  typedef VectorType Normal;
};
typedef TriMesh_ArrayKernelT<MyTraits>  MyMesh;


//-----------------------------------------------------------------------------



void usage(const char* _argv0)
{
  std::cerr << "\n\nUsage: \n"
	    << _argv0 << "  <-e | -m> <-a angle> <-r resolution> <-o filename> \n\n";
  
  std::cerr << "  -e   Use Extended Marching Cubes (default)\n"
	    << "  -m   Use standard Marching Cubes\n"
	    << "  -a   Feature detection threshold\n"
	    << "  -r   Grid resolution (default is 50)\n"
	    << "  -o   Write result to filename (should be *.{off,obj,stl}), "
	    << "defaults to output.off\n"
	    << "\n";

  exit(1);
}


//-----------------------------------------------------------------------------


int main(int argc, char** argv)
{
  // parameters
  const char*       filename = "output.off";
  const char*       inputfilename = "data.txt";
  unsigned int      res      = 50;
  unsigned int      res_x    = 390;
  unsigned int      res_y    = 252;
  unsigned int      res_z    = 225;
  enum { MC, EMC }  mode     = EMC;
  float             angle    = 30.0;


  
  // parse command line
  int         c;
  extern char *optarg;
  //extern int  optind;
  std::cout <<"argc "<< argc << std::endl;
  for (int i=1; i< argc; i++){
   std::cout <<"argv["<< i <<"] = "<< argv[i] << std::endl;
   if ( argv[i] == std::string("-a")){angle = atof(argv[++i]);}
   else if ( argv[i] == std::string("-e")){mode = EMC;}
   else if ( argv[i] == std::string("-m")){mode = MC;}
   else if ( argv[i] == std::string("-i")){inputfilename = argv[++i];}
   else if ( argv[i] == std::string("-o")){filename= argv[++i];}
   else if ( argv[i] == std::string("-r")){res_x= atoi(argv[++i]);
                                           res_y= atoi(argv[++i]);
                                           res_z= atoi(argv[++i]);}

   else if ( argv[i] == std::string("-h")){usage(argv[0]); exit(0);}
   else { std::cerr <<"un recognised. \n"<< argv[i] << std::endl;
	usage( argv[0]);
   }
  }
  

  // output parameters
  switch (mode)
  {
    case MC:  
      std::cout << "Standard Marching Cubes\n"; 
      break;

    case EMC: 
      std::cout << "Extended Marching Cubes\n"
		<< "Feature detection angle: " << angle 
		<< std::endl;
      break;
  }
  std::cout << "Grid: " << res_x << "x" << res_y << "x" << res_z << std::endl;
  std::cout << "Output file: " << filename << std::endl;
  std::cout << "Input file: " << inputfilename << std::endl;

  VectorType origin(0,0,0);
  VectorType axis_x(1,0,0);
  VectorType axis_y(0,1,0);
  VectorType axis_z(0,0,1);

  ScalarGridT<float, VectorType> grid(origin, axis_x, axis_y, axis_z, res_x, res_y, res_z);
  std::cout << "Reading " << inputfilename << "..." << std::endl;

  // read in 
  std::ifstream inputfile(inputfilename);
  std::string line;
  char delim = ',';
  if(inputfile.is_open())
  {
      for(unsigned int x=0; x<res_x; x++)
      {
          for(unsigned int y=0; y<res_y; y++)
          {
              for(unsigned int z=0; z<res_z; z++)
              {
                  //inputfile << grid(x,y,z) << "," << clusteredNormals(x,y,z).n1[0] << "," << clusteredNormals(x,y,z).n1[1] << "," << clusteredNormals(x,y,z).n1[2] << std::endl;
                std::getline(inputfile, line);
                std::vector<std::string> strdata = split(line, delim);
                float data = std::strtof((strdata.at(0)).c_str(),0);
                grid(x,y,z) = -1.0 * data;
                VectorType p0 = grid.point(x,y,z);
              }
          }
      }
      inputfile.close();
      std::cout << "Successfully read in grid data." << std::endl;
  }
  else std::cout << "Unable to open file " << inputfilename << std::endl;

  MyMesh  mesh;
/*
  // extract 0-level isosurface
  MyMesh  mesh;
  switch (mode)
  {
    case MC:  
      grid.build_scalar_distance_cache();
      marching_cubes(grid, mesh); 
      break;

    case EMC:
      grid.build_is_inside_cache();
      extended_marching_cubes(grid, mesh, angle);
      break;
  }
*/

 if(mode == EMC){
   extended_marching_cubes(grid, mesh, angle);
 }
 else{
   marching_cubes(grid, mesh);
 }
  
  // write result
  write_mesh(mesh, filename);
  std::cout << "Written mesh to outputfile: " << filename << ". Done." << std::endl;

  
  return 0;
}


//-----------------------------------------------------------------------------
