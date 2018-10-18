#ifndef GLCOMMON_H
#define GLCOMMON_H

#include <d3d_base/exception.h>
#include <sstream>

using std::ostringstream;

namespace D3D
{
#define D3D_GLEW_CHECKED_CALL(glew_call)                                                    \
{                                                                                       \
    GLenum err = glew_call                                                          \
    if( err != GLEW_OK)                                                             \
    {                                                                                   \
        /* generate message */                                                          \
        ostringstream os;                                                               \
        os << "GLEW Error: " << (const char*)glewGetErrorString(err);                                \
        throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, os.str().c_str());     \
    }                                                                                   \
}

#define D3D_GL_CHECK_FBO(fboID)                                                                                             \
{                                                                                                                       \
    /* check if asked buffer is bound  */                                                                               \
    GLint boundFboID;                                                                                                   \
    glGetIntegerv(GL_FRAMEBUFFER_BINDING_EXT, &boundFboID);                                                             \
    if(fboID != (unsigned) boundFboID)                                                                                            \
    {                                                                                                                   \
        throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, "FBO is not bound");                                     \
    }                                                                                                                   \
    /* check for framebuffer errors */                                                                                  \
    GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);                                                    \
    switch(status)                                                                                                      \
    {                                                                                                                   \
    case GL_FRAMEBUFFER_COMPLETE_EXT:  /* Everything's OK */                                                            \
        break;                                                                                                          \
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT:                                                                      \
        throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT_EXT");           \
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT:                                                              \
        throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT_EXT");   \
    case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:                                                                      \
        throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, "GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT");           \
    case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:                                                                         \
        throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, "GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT");              \
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT:                                                                     \
        throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER_EXT");          \
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT:                                                                     \
        throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER_EXT");          \
    case GL_FRAMEBUFFER_UNSUPPORTED_EXT:                                                                                \
        throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, "GL_FRAMEBUFFER_UNSUPPORTED_EXT");                     \
    default:                                                                                                            \
        throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, "Unknown Error ");                                     \
    }                                                                                                                   \
}

#define D3D_GL_CHECK_SHADER(shaderID) \
{ \
    GLint compiled; \
glGetShaderiv(shaderID, GL_COMPILE_STATUS, &compiled); \
 \
if (!compiled) \
{ \
    GLint length; \
    GLchar* log; \
    glGetShaderiv(vertShader, GL_INFO_LOG_LENGTH, &length); \
\
    log = (GLchar*) malloc(length); \
    glGetShaderInfoLog(vertShader, length, &length, log); \
    ostringstream os;                                                               \
    os << "GL Shader Compilation Error: " << log;                                 \
    free(log); \
    throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, os.str().c_str()); \
} \
}

#define D3D_GL_CHECK_PROGRAM(programID) \
{ \
GLint linked; \
glGetProgramiv(shaderProgram, GL_LINK_STATUS, &linked); \
 \
if (!linked) \
{ \
    GLint length; \
    GLchar* log; \
    glGetProgramiv(shaderProgram, GL_INFO_LOG_LENGTH, &length); \
\
    log = (GLchar*) malloc(length); \
    glGetProgramInfoLog(shaderProgram, length, &length, log); \
    ostringstream os;                                                               \
    os << "GL Program Linker Error: " << log;                                 \
    free(log); \
    throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, os.str().c_str()); \
} \
}

#define D3D_GL_CHECK_ERROR \
{ \
    /* error handling */ \
    GLenum errCode; \
    while ((errCode = glGetError()) != GL_NO_ERROR) \
    { \
        ostringstream os;                                   \
        os << "OpenGL Error: " << gluErrorString(errCode);      \
        throw D3D::Exception(__FILE__, __LINE__, __PRETTY_FUNCTION__, os.str().c_str()); \
    } \
}

}
#endif

