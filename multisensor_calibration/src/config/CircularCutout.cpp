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

#include "../include/multisensor_calibration/config/CircularCutout.h"

namespace multisensor_calibration
{

//==================================================================================================
CircularCutout::CircularCutout() :
  Cutout(1, 3)
{
}

//==================================================================================================
CircularCutout::CircularCutout(const std::vector<float> iCoefficients) :
  Cutout(1, 3, iCoefficients)
{
}

//==================================================================================================
CircularCutout::~CircularCutout()
{
}

//==================================================================================================
float CircularCutout::getRadius() const
{
    return getCoefficients()[2];
}

//==================================================================================================
float CircularCutout::getCenterX() const
{
    return getCoefficients()[0];
}

//==================================================================================================
float CircularCutout::getCenterY() const
{
    return getCoefficients()[1];
}

//==================================================================================================
bool CircularCutout::isPointInside(const float x, const float y,
                                   float* opDistance, float* opPenalty) const
{
    cv::Vec2d centerPnt = cv::Vec2d(getCenterX(), getCenterY());
    double radius       = getRadius();
    double norm         = cv::norm(cv::Vec2d(x, y), centerPnt, cv::NORM_L2);
    bool isInside       = (norm <= radius) ? true : false;

    if (opDistance)
        *opDistance = static_cast<float>(norm);

    if (opPenalty)
        *opPenalty = 10.f;
    // *opPenalty = -(5.f / radius) * norm + 6.f;
    // *opPenalty = 1.f - (1.f / radius) * static_cast<float>(std::log(norm / radius));

    return isInside;
}

} // namespace multisensor_calibration