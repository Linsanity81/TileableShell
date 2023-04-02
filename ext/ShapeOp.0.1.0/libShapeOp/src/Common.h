///////////////////////////////////////////////////////////////////////////////
// This file is part of ShapeOp, a lightweight C++ library
// for static and dynamic geometry processing.
//
// Copyright (C) 2014 Sofien Bouaziz <sofien.bouaziz@gmail.com>
// Copyright (C) 2014 LGG EPFL
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
///////////////////////////////////////////////////////////////////////////////
#ifndef COMMON_H
#define COMMON_H
///////////////////////////////////////////////////////////////////////////////
/** \file
This file is used to define macros and typedefs used by the C++ code and the C API.*/
///////////////////////////////////////////////////////////////////////////////
/** \brief Defines the scalar type used by the ShapeOp solver (float or double).*/
#ifndef SHAPEOP_SCALAR
#define SHAPEOP_SCALAR double
#endif
/** \brief Defines the scalar type used by the ShapeOp solver (float or double).*/
typedef SHAPEOP_SCALAR ShapeOpScalar;
///////////////////////////////////////////////////////////////////////////////
/** \brief Defines the API prefix for the current platform.*/
#if defined(_WIN32) || defined(_WIN64)
#pragma warning( disable : 4251 ) //Disable warnings about templates and std types exposed in the c++ interface.
#ifdef SHAPEOP_EXPORT
#define SHAPEOP_API __declspec(dllexport)
#else
#define SHAPEOP_API __declspec(dllimport)
#endif
#else
#define SHAPEOP_API
#endif
///////////////////////////////////////////////////////////////////////////////
/** \brief A macro to inline all code which produces a header-only library.*/
#ifdef SHAPEOP_HEADER_ONLY
#define SHAPEOP_INLINE inline
#else
#define SHAPEOP_INLINE
#endif
///////////////////////////////////////////////////////////////////////////////
#endif // COMMON_H
///////////////////////////////////////////////////////////////////////////////
