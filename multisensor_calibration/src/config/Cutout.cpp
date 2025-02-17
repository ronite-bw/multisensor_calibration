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

#include "../include/multisensor_calibration/config/Cutout.h"

#include "../include/multisensor_calibration/common/common.h"

namespace multisensor_calibration
{

//==================================================================================================
Cutout::Cutout(const int iId, const int iNumCoefficients) :
  Cutout(iId, iNumCoefficients, {})
{
}

//==================================================================================================
Cutout::Cutout(const int iId, const int iNumCoefficients, const std::vector<float> iCoefficients) :
  geometryId_(iId),
  numCoefficients_(iNumCoefficients),
  coefficients_(iCoefficients)
{
    assert(iCoefficients.empty() || iCoefficients.size() == static_cast<uint>(iNumCoefficients));
}

//==================================================================================================
Cutout::~Cutout()
{
}

//==================================================================================================
int Cutout::getGeometryId() const
{
    return geometryId_;
}

//==================================================================================================
int Cutout::getNumCoefficients() const
{
    return numCoefficients_;
}

//==================================================================================================
std::vector<float> Cutout::getCoefficients() const
{
    return coefficients_;
}

//==================================================================================================
float Cutout::getRadius() const
{
    return 0.f;
}

//==================================================================================================
float Cutout::getCenterX() const
{
    return 0.f;
}

//==================================================================================================
float Cutout::getCenterY() const
{
    return 0.f;
}

//==================================================================================================
void Cutout::setCoefficients(const std::vector<float>& iCoefficients)
{
    assert(iCoefficients.size() == static_cast<uint>(numCoefficients_));

    coefficients_ = iCoefficients;
}

//==================================================================================================
bool Cutout::isPointInside(const cv::Vec2f iPnt,
                           float* opDistance, float* opPenalty) const
{
    UNUSED_VAR(opDistance);
    UNUSED_VAR(opPenalty);

    return isPointInside(iPnt(0), iPnt(1));
}

} // namespace multisensor_calibration