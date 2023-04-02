///////////////////////////////////////////////////////////////
//
// libigl_UI.h
//
//   User interface with libigl
//
// by Yucheng Sun and Peng SONG
//
// 26/Nov/2020
//
//
///////////////////////////////////////////////////////////////


#ifndef INTERLOCKSHELL_LIBIGL_UI_H
#define INTERLOCKSHELL_LIBIGL_UI_H

//#include <igl/opengl/glfw/Viewer.h>

namespace igl
{namespace opengl
    {namespace glfw {
            class Viewer;
        }}}

//// manage all UI here
void setViewerUI(igl::opengl::glfw::Viewer &viewer);


#endif //INTERLOCKSHELL_LIBIGL_UI_H