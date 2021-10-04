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
#include <math.h>
#include <Eigen/Dense>
#include <pdal/io/LasWriter.hpp>
#include <pdal/io/BufferReader.hpp>
#include <SRITrajectory.hpp>

namespace pdal
{

using namespace Dimension;
using namespace Eigen;

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
    args.add("maximum_incidence_angle", "Maximum allowable incidence angle (degrees <90)", m_maximumIncidenceAngle, 85.0);
    args.add("include_incidence_angle", "Include incidence angle in TPU computation", m_includeIncidenceAngle);
}


void ALS_TPU::addDimensions(PointLayoutPtr layout)
{
    m_lidarDist = layout->registerOrAssignDim("LidarDistance", Type::Double);
    m_scanAngle = layout->registerOrAssignDim("MyScanAngle", Type::Double);
    m_incAngle = layout->registerOrAssignDim("IncAngle", Type::Double);
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
        // 1. interpolate sensor xyz and heading from trajectory points
        double trajX, trajY, trajZ, trajHeading;
        bool successfulInterp = linearInterpolation(
            cloud->getFieldAs<double>(Id::GpsTime, i),
            trajX, trajY, trajZ, trajHeading,
            trajectory, interpIdx
        );
        // // test
        // if (successfulInterp)
        // {
        //     cloud->setField(Id::X, i, trajX);
        //     cloud->setField(Id::Y, i, trajY);
        //     cloud->setField(Id::Z, i, trajZ);
        //     cloud->setField(Id::Azimuth, i, trajHeading);
        // }

        // 2. invert for lidar distance and laser scan angle; estimate incidence angle
        double lidarDist, scanAngle, incidenceAngle;
        invertObservations(
            cloud->point(i),
            trajX, trajY, trajZ, trajHeading,
            lidarDist, scanAngle, incidenceAngle
        );
        // test
        cloud->setField(m_lidarDist, i, lidarDist);
        cloud->setField(m_scanAngle, i, scanAngle);
        cloud->setField(m_incAngle, i, incidenceAngle);


        // 3. build observation covariance matrix



        // 4. zero out items that we do not estimate (roll, pitch, forward/back scan angle)



        // 5. propagate observation variance into point xyz covariance matrix



        //6. store covariance information in new dimensions
    }


    // savePoints("cloud.las", cloud);
    // savePoints("trajectory.las", trajectory);
    return cloud;
}


void ALS_TPU::invertObservations(
    PointRef cloudPoint,
    double trajX, double trajY, double trajZ, double trajHeading,
    double& lidarDist, double& scanAngle, double& incidenceAngle)
{
    // lidar distance
    Vector3d laserVector;
    laserVector << (cloudPoint.getFieldAs<double>(Id::X) - trajX),
                   (cloudPoint.getFieldAs<double>(Id::Y) - trajY),
                   (cloudPoint.getFieldAs<double>(Id::Z) - trajZ);
    lidarDist = laserVector.norm();

    // laser to ground surface incidence angle
    Vector3d normalVector;
    normalVector << cloudPoint.getFieldAs<double>(Id::NormalX),
                    cloudPoint.getFieldAs<double>(Id::NormalY),
                    cloudPoint.getFieldAs<double>(Id::NormalZ);
    laserVector /= lidarDist;
    incidenceAngle = acos(laserVector.dot(normalVector)) * 180.0 / M_PI;
    if (incidenceAngle > m_maximumIncidenceAngle)
    {
        incidenceAngle = m_maximumIncidenceAngle;
    }

    // scan angle
    Vector3d nadirVector, headingVector, crossProduct;
    trajHeading *= M_PI / 180.0;
    nadirVector << 0.0, 0.0, -1.0;
    headingVector << sin(trajHeading), cos(trajHeading), 0.0;
    scanAngle = acos(laserVector.dot(nadirVector)) * 180.0 / M_PI;
    crossProduct = laserVector.cross(headingVector);
    scanAngle = copysign(scanAngle, crossProduct(2));
}


bool ALS_TPU::linearInterpolation(
    double pointTime,
    double& trajX, double& trajY, double& trajZ, double& trajHeading,
    PointViewPtr trajectory, PointId& interpIdx)
{
    // special case: before left end
    if (pointTime < trajectory->getFieldAs<double>(Id::GpsTime, 0))
    {
        interpIdx = 0;
        return false;
    }

    // special case: beyond right end
    if (pointTime > trajectory->getFieldAs<double>(Id::GpsTime, trajectory->size() - 1))
    {
        interpIdx = trajectory->size() - 1;
        return false;
    }

    // find left end of interpolation interval
    while (pointTime > trajectory->getFieldAs<double>(Id::GpsTime, interpIdx + 1))
    {
        interpIdx++;
    }
    while (pointTime < trajectory->getFieldAs<double>(Id::GpsTime, interpIdx))
    {
        interpIdx--;
    }

    // interpolate
    double factor = (pointTime - trajectory->getFieldAs<double>(Id::GpsTime, interpIdx))
                  / (trajectory->getFieldAs<double>(Id::GpsTime, interpIdx + 1)
                  - trajectory->getFieldAs<double>(Id::GpsTime, interpIdx));

    trajX = trajectory->getFieldAs<double>(Id::X, interpIdx)
       + factor * (trajectory->getFieldAs<double>(Id::X, interpIdx + 1)
       - trajectory->getFieldAs<double>(Id::X, interpIdx));

    trajY = trajectory->getFieldAs<double>(Id::Y, interpIdx)
       + factor * (trajectory->getFieldAs<double>(Id::Y, interpIdx + 1)
       - trajectory->getFieldAs<double>(Id::Y, interpIdx));

    trajZ = trajectory->getFieldAs<double>(Id::Z, interpIdx)
       + factor * (trajectory->getFieldAs<double>(Id::Z, interpIdx + 1)
       - trajectory->getFieldAs<double>(Id::Z, interpIdx));

    trajHeading = trajectory->getFieldAs<double>(Id::Azimuth, interpIdx)
       + factor * (trajectory->getFieldAs<double>(Id::Azimuth, interpIdx + 1)
       - trajectory->getFieldAs<double>(Id::Azimuth, interpIdx));

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
