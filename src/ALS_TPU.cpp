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
#include <pdal/io/LasWriter.hpp>
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
    PointViewSet viewSet;
    if (this->m_cloud)
    {
        PointViewPtr result = this->tpu(this->m_cloud, view);
        viewSet.insert(result);
        this->m_complete = true;
    }
    else
    {
        this->m_cloud = view;
    }
    return viewSet;
}


void ALS_TPU::done(PointTableRef _)
{
    if (!this->m_complete)
    {
        throw pdal_error(
            "filters.als_tpu must have two point view inputs, no more, no less");
    }
}


PointViewPtr ALS_TPU::tpu(PointViewPtr cloud, PointViewPtr trajectory)
{
    PointId interpIdx = 0;

    // iterate over each cloud point
    for (PointId i = 0; i < cloud->size(); ++i)
    {
        // 1. interpolate sensor xyz and heading
        double ix, iy, iz, ih;
        bool successfulInterp = linearInterpolation(
            cloud->getFieldAs<double>(Dimension::Id::GpsTime, i),
            ix, iy, iz, ih,
            trajectory, interpIdx
        );
        // // test
        // if (successfulInterp)
        // {
        //     cloud->setField(Dimension::Id::X, i, ix);
        //     cloud->setField(Dimension::Id::Y, i, iy);
        //     cloud->setField(Dimension::Id::Z, i, iz);
        //     cloud->setField(Dimension::Id::Azimuth, i, ih);
        // }

        // 2. invert for lidar distance and laser scan angle; estimate incidence angle



        // 3. build observation covariance matrix



        // 4. zero out items that we do not estimate (roll, pitch, forward/back scan angle)



        // 5. propagate observation variance into point xyz covariance matrix



        //6. store covariance information in new dimensions
    }


    savePoints("cloud.las", cloud);
    savePoints("trajectory.las", trajectory);
    return cloud;
}


bool ALS_TPU::linearInterpolation(
    double t,
    double& ix, double& iy, double& iz, double& ih,
    PointViewPtr trajectory, PointId& interpIdx)
{
    // special case: before left end
    if (t < trajectory->getFieldAs<double>(Dimension::Id::GpsTime, 0))
    {
        interpIdx = 0;
        return false;
    }

    // special case: beyond right end
    if (t > trajectory->getFieldAs<double>(Dimension::Id::GpsTime, trajectory->size() - 1))
    {
        interpIdx = trajectory->size() - 1;
        return false;
    }

    // find left end of interpolation interval
    while (t > trajectory->getFieldAs<double>(Dimension::Id::GpsTime, interpIdx + 1))
    {
        interpIdx++;
    }
    while (t < trajectory->getFieldAs<double>(Dimension::Id::GpsTime, interpIdx))
    {
        interpIdx--;
    }

    // interpolate
    double factor = (t - trajectory->getFieldAs<double>(Dimension::Id::GpsTime, interpIdx))
                  / (trajectory->getFieldAs<double>(Dimension::Id::GpsTime, interpIdx + 1)
                  - trajectory->getFieldAs<double>(Dimension::Id::GpsTime, interpIdx));

    ix = trajectory->getFieldAs<double>(Dimension::Id::X, interpIdx)
       + factor * (trajectory->getFieldAs<double>(Dimension::Id::X, interpIdx + 1)
       - trajectory->getFieldAs<double>(Dimension::Id::X, interpIdx));

    iy = trajectory->getFieldAs<double>(Dimension::Id::Y, interpIdx)
       + factor * (trajectory->getFieldAs<double>(Dimension::Id::Y, interpIdx + 1)
       - trajectory->getFieldAs<double>(Dimension::Id::Y, interpIdx));

    iz = trajectory->getFieldAs<double>(Dimension::Id::Z, interpIdx)
       + factor * (trajectory->getFieldAs<double>(Dimension::Id::Z, interpIdx + 1)
       - trajectory->getFieldAs<double>(Dimension::Id::Z, interpIdx));

    ih = trajectory->getFieldAs<double>(Dimension::Id::Azimuth, interpIdx)
       + factor * (trajectory->getFieldAs<double>(Dimension::Id::Azimuth, interpIdx + 1)
       - trajectory->getFieldAs<double>(Dimension::Id::Azimuth, interpIdx));

    return true;
}


void ALS_TPU::savePoints(std::string filename, PointViewPtr view)
{
    pdal::Options ops;
    ops.add("filename", filename);
    ops.add("minor_version", 4);
    ops.add("extra_dims", "all");

    BufferReader bufferReader;
    bufferReader.addView(view);

    pdal::LasWriter writer;
    writer.setOptions(ops);
    writer.setInput(bufferReader);
    writer.prepare(view->table());
    writer.execute(view->table());
}

} // namespace pdal
