// Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef LIB3D_CORE_VERSION_HPP
#define LIB3D_CORE_VERSION_HPP

#define LIB3D_CORE_VERSION_STR 2.0.1.47.fb85c91
#define LIB3D_CORE_VERSION_MAJOR 2
#define LIB3D_CORE_VERSION_MINOR 0
#define LIB3D_CORE_VERSION_PATCH 1

// std
#include <string>

#include "common.h"

/**
 @namespace lib3d::core
 @brief Base namespace of lib3D_core.
 */
namespace lib3d
{
namespace core
{

/**
 @brief Function to get current version string of library.
 */
inline std::string getVersionStr()
{
    return std::string(LIB3D_TOSTRING(LIB3D_CORE_VERSION_STR));
}

/**
 @brief Function to get current version major number of library.
 */
inline int getVersionMajor()
{
    return LIB3D_CORE_VERSION_MAJOR;
}

/**
 @brief Function to get current version minor number of library.
 */
inline int getVersionMinor()
{
    return LIB3D_CORE_VERSION_MINOR;
}

/**
 @brief Function to get current version path number of library.
 */
inline int getVersionPatch()
{
    return LIB3D_CORE_VERSION_PATCH;
}

} // namespace core
} // namespace lib3d

#endif // LIB3D_CORE_VERSION_HPP
