#ifndef MESHTODEPTHMAPRENDERER_H
#define MESHTODEPTHMAPRENDERER_H

#include <GL/glut.h>
#include <vector>
#include <Eigen/Dense>
#include <d3d_base/cameraMatrix.h>
#include <d3d_base/depthMap.h>

namespace D3D
{
template<typename T, typename U>
class MeshToDepthMapRenderer
{
public:
    MeshToDepthMapRenderer(int width, int height, U minDepth, U maxDepth);
    ~MeshToDepthMapRenderer();
    void uploadMesh(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::vector<int> >& faces);
    D3D::DepthMap<T, U> renderFromViewpoint(const CameraMatrix<U>& cam);

private:
    void initializeGL();
    void setUpProjection(const CameraMatrix<U>& cam);

    void readResultBack(float* dataPtr);
    void readResultBack(double* dataPtr);

    int width;
    int height;

    U minDepth;
    U maxDepth;

    GLuint fbo;
    GLuint depthbuffer;
    GLuint colorbuffer;
    GLuint fragShader;
    GLuint vertShader;
    GLuint shaderProgram;
    GLuint meshDisplayList;
};
}


#endif // MESHTODEPTHMAPRENDERER_H
