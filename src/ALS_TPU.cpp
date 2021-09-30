/******************************************************************************
 * Copyright (c) 2021, Preston J Hartzell (preston.hartzell@gmail.com)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following
 * conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Hobu, Inc. nor the
 *       names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior
 *       written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 ****************************************************************************/

#include "ALS_TPU.hpp"

#include <SRITrajectory.hpp>
#include <pdal/io/BufferReader.hpp>


namespace pdal
{

using namespace Dimension;

static PluginInfo const s_info
{
    "filters.als_tpu",
    "Per-point airborne lidar Total Propagated Uncertainty (TPU) via a generic sensor model.",
    "http://link/to/documentation"
};

CREATE_SHARED_STAGE(ALS_TPU, s_info)

std::string ALS_TPU::getName() const
{
    return s_info.name;
}

void ALS_TPU::addArgs(ProgramArgs& args)
{
    args.add("s_lidar_distance", "Lidar distance standard deviation (meters)", m_sLidarDistance);
    args.add("s_scan_angle", "Scan angle standard deviation (degrees)", m_sScanAngle);
    args.add("s_sensor_xy", "Sensor x and y position standard deviation (meters)", m_sSensorXY);
    args.add("s_sensor_z", "Sensor z position standard deviation (meters)", m_sSensorZ);
    args.add("s_sensor_rollpitch", "Sensor roll and pitch standard deviation (degrees)", m_sSensorRollPitch);
    args.add("s_sensor_yaw", "Sensor yaw standard deviation (degrees)", m_sSensorYaw);
    args.add("laser_beam_divergence", "Laser beam divergence (milliradians)", m_laserBeamDivergence);
    args.add("maximum_incidence_angle", "Maximum allowable incidence angle (degrees <90)", m_maximumIncidenceAngle);
    args.add("include_incidence_angle", "Include incidence angle in TPU computation", m_includeIncidenceAngle);
}


PointViewSet ALS_TPU::run(PointViewPtr view)
{
    PointViewPtr trajectory = estimatedTrajectory(view);

    PointViewSet viewSet;
    viewSet.insert(trajectory);

    return viewSet;
}


PointViewPtr ALS_TPU::estimatedTrajectory(PointViewPtr view)
{
    SRITrajectory filter;

    BufferReader reader;
    reader.addView(view);
    filter.setInput(reader);

    PointTable table;
    filter.prepare(table);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr newView = *viewSet.begin();

    return newView;
}

} // namespace pdal
