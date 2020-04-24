///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020, JSK.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/* Author: Moju Zhao
   Desc:   Noise model
*/

#pragma once

namespace
{
  unsigned int noise_seed = 0;
}

namespace gazebo
{
  double gaussianKernel(double sigma)
  {
    // generation of two normalized uniform random variables
    double U1 = static_cast<double>(rand_r(&noise_seed)) / static_cast<double>(RAND_MAX);
    double U2 = static_cast<double>(rand_r(&noise_seed)) / static_cast<double>(RAND_MAX);

    // using Box-Muller transform to obtain a varaible with a standard normal distribution
    double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

    // scaling
    Z0 = sigma * Z0;
    return Z0;
  }

  double addNoise(double& current_drift, double drift, double drift_frequency, double offset, double gaussian_noise, double dt)
  {
    current_drift = exp(-dt * drift_frequency) * current_drift + dt * gaussianKernel(sqrt( 2 * drift_frequency) * drift);
    return offset + current_drift + gaussianKernel(gaussian_noise);
  }
};
