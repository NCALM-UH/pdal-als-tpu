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
    m_xVar = layout->registerOrAssignDim("VarianceX", Type::Float);
    m_yVar = layout->registerOrAssignDim("VarianceY", Type::Float);
    m_zVar = layout->registerOrAssignDim("VarianceZ", Type::Float);
    m_xyCov = layout->registerOrAssignDim("CovarianceXY", Type::Float);
    m_xzCov = layout->registerOrAssignDim("CovarianceXZ", Type::Float);
    m_yzCov = layout->registerOrAssignDim("CovarianceYZ", Type::Float);
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

        // 2. invert for lidar distance and laser scan angle (left-right); estimate incidence angle
        double lidarDist, scanAngleLR, incidenceAngle;
        invertObservations(
            cloud->point(i),
            trajX, trajY, trajZ, trajHeading,
            lidarDist, scanAngleLR, incidenceAngle
        );
        // test
        cloud->setField(m_lidarDist, i, lidarDist);
        cloud->setField(m_scanAngle, i, scanAngleLR);
        cloud->setField(m_incAngle, i, incidenceAngle);

        // 3. build observation covariance matrix
        MatrixXd obsCovariance = observationCovariance(
            lidarDist,
            scanAngleLR,
            incidenceAngle);

        // 4. zero out items that we do not estimate (roll, pitch, forward/back scan angle)
        double trajRoll = 0.0;
        double trajPitch = 0.0;
        double scanAngleFB = 0.0;

        // 5. propagate observation variance into point xyz covariance matrix
        Matrix3d lidarPointCovariance = propagateCovariance(
            lidarDist, scanAngleLR, scanAngleFB,
            trajX, trajY, trajZ,
            trajRoll, trajPitch, trajHeading,
            obsCovariance
        );

        //6. store covariance information in new dimensions
        cloud->setField(m_xVar, i, lidarPointCovariance(0, 0));
        cloud->setField(m_yVar, i, lidarPointCovariance(1, 1));
        cloud->setField(m_zVar, i, lidarPointCovariance(2, 2));
        cloud->setField(m_xyCov, i, lidarPointCovariance(0, 1));
        cloud->setField(m_xzCov, i, lidarPointCovariance(0, 2));
        cloud->setField(m_yzCov, i, lidarPointCovariance(1, 2));
    }

    // savePoints("cloud.las", cloud);
    // savePoints("trajectory.las", trajectory);
    return cloud;
}


Matrix3d ALS_TPU::propagateCovariance(
    double lidarDist, double scanAngleLR, double scanAngleFB,
    double trajX, double trajY, double trajZ,
    double trajRoll, double trajPitch, double trajHeading,
    MatrixXd obsCovariance
)
{
    double trajYaw = -trajHeading;  // heading to yaw
    scanAngleLR *= M_PI / 180.0;    // degrees to radians
    scanAngleFB *= M_PI / 180.0;    // degrees to radians
    trajRoll *= M_PI / 180.0;       // degrees to radians
    trajPitch *= M_PI / 180.0;      // degrees to radians
    trajYaw *= M_PI / 180.0;        // degrees to radians

    // Jacobian matrix ordered the same as the observation covariance matrix
    MatrixXd a = MatrixXd::Zero(3, 9);
    // partials with respect to lidar point X
    a(0, 0) = (sin(trajPitch)*sin(trajRoll)*sin(trajY) + cos(trajRoll)*cos(trajY))*sin(scanAngleLR)*cos(scanAngleFB) + (-sin(trajPitch)*sin(trajY)*cos(trajRoll) + sin(trajRoll)*cos(trajY))*cos(scanAngleFB)*cos(scanAngleLR) + sin(scanAngleFB)*sin(trajY)*cos(trajPitch);
    a(0, 1) = lidarDist*(sin(trajPitch)*sin(trajRoll)*sin(trajY) + cos(trajRoll)*cos(trajY))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(trajPitch)*sin(trajY)*cos(trajRoll) - sin(trajRoll)*cos(trajY))*sin(scanAngleLR)*cos(scanAngleFB);
    a(0, 2) = -lidarDist*(sin(trajPitch)*sin(trajRoll)*sin(trajY) + cos(trajRoll)*cos(trajY))*sin(scanAngleFB)*sin(scanAngleLR) + lidarDist*(sin(trajPitch)*sin(trajY)*cos(trajRoll) - sin(trajRoll)*cos(trajY))*sin(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(trajY)*cos(trajPitch)*cos(scanAngleFB);
    a(0, 3) = 1;
    a(0, 4) = 0;
    a(0, 5) = 0;
    a(0, 6) = -lidarDist*(-sin(trajPitch)*sin(trajRoll)*sin(trajY) - cos(trajRoll)*cos(trajY))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(trajPitch)*sin(trajY)*cos(trajRoll) - sin(trajRoll)*cos(trajY))*sin(scanAngleLR)*cos(scanAngleFB);
    a(0, 7) = -lidarDist*sin(trajPitch)*sin(scanAngleFB)*sin(trajY) + lidarDist*sin(trajRoll)*sin(scanAngleLR)*sin(trajY)*cos(trajPitch)*cos(scanAngleFB) - lidarDist*sin(trajY)*cos(trajPitch)*cos(trajRoll)*cos(scanAngleFB)*cos(scanAngleLR);
    a(0, 8) = lidarDist*(sin(trajPitch)*sin(trajRoll)*cos(trajY) - sin(trajY)*cos(trajRoll))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(trajPitch)*cos(trajRoll)*cos(trajY) + sin(trajRoll)*sin(trajY))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*cos(trajPitch)*cos(trajY);
    // partials with respect to lidar point Y
    a(1, 0) = (sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*sin(scanAngleLR)*cos(scanAngleFB) + (-sin(trajPitch)*cos(trajRoll)*cos(trajYaw) - sin(trajRoll)*sin(trajYaw))*cos(scanAngleFB)*cos(scanAngleLR) + sin(scanAngleFB)*cos(trajPitch)*cos(trajYaw);
    a(1, 1) = lidarDist*(sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*sin(scanAngleLR)*cos(scanAngleFB);
    a(1, 2) = -lidarDist*(sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*sin(scanAngleFB)*sin(scanAngleLR) + lidarDist*(sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*sin(scanAngleFB)*cos(scanAngleLR) + lidarDist*cos(trajPitch)*cos(scanAngleFB)*cos(trajYaw);
    a(1, 3) = 0;
    a(1, 4) = 1;
    a(1, 5) = 0;
    a(1, 6) = -lidarDist*(-sin(trajPitch)*sin(trajRoll)*cos(trajYaw) + sin(trajYaw)*cos(trajRoll))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*sin(scanAngleLR)*cos(scanAngleFB);
    a(1, 7) = -lidarDist*sin(trajPitch)*sin(scanAngleFB)*cos(trajYaw) + lidarDist*sin(trajRoll)*sin(scanAngleLR)*cos(trajPitch)*cos(scanAngleFB)*cos(trajYaw) - lidarDist*cos(trajPitch)*cos(trajRoll)*cos(scanAngleFB)*cos(scanAngleLR)*cos(trajYaw);
    a(1, 8) = lidarDist*(-sin(trajPitch)*sin(trajRoll)*sin(trajYaw) - cos(trajRoll)*cos(trajYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(-sin(trajPitch)*sin(trajYaw)*cos(trajRoll) + sin(trajRoll)*cos(trajYaw))*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*sin(scanAngleFB)*sin(trajYaw)*cos(trajPitch);
    // partials with respect to lidar point Z
    a(2, 0) = -sin(trajPitch)*sin(scanAngleFB) + sin(trajRoll)*sin(scanAngleLR)*cos(trajPitch)*cos(scanAngleFB) - cos(trajPitch)*cos(trajRoll)*cos(scanAngleFB)*cos(scanAngleLR);
    a(2, 1) = lidarDist*sin(trajRoll)*cos(trajPitch)*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleLR)*cos(trajPitch)*cos(trajRoll)*cos(scanAngleFB);
    a(2, 2) = -lidarDist*sin(trajPitch)*cos(scanAngleFB) - lidarDist*sin(trajRoll)*sin(scanAngleFB)*sin(scanAngleLR)*cos(trajPitch) + lidarDist*sin(scanAngleFB)*cos(trajPitch)*cos(trajRoll)*cos(scanAngleLR);
    a(2, 3) = 0;
    a(2, 4) = 0;
    a(2, 5) = 1;
    a(2, 6) = lidarDist*sin(trajRoll)*cos(trajPitch)*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleLR)*cos(trajPitch)*cos(trajRoll)*cos(scanAngleFB);
    a(2, 7) = -lidarDist*sin(trajPitch)*sin(trajRoll)*sin(scanAngleLR)*cos(scanAngleFB) + lidarDist*sin(trajPitch)*cos(trajRoll)*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*sin(scanAngleFB)*cos(trajPitch);
    a(2, 8) = 0;

    // propagate
    Matrix3d c = a * obsCovariance * a.transpose();

    return c;
}



MatrixXd ALS_TPU::observationCovariance(
    double lidarDist, double scanAngle, double incidenceAngle)
{
    /*
    Variance order: 
        1. lidar distance
        2. scan angle (left-right)
        3. scan angle (forward-back)
        4. sensor x
        5. sensor y
        6. sensor z
        7. sensor roll
        8. sensor pitch
        9. sensor yaw

    Beam divergence uncertainty:
        1. Applied to the lidar distance when including the incidence angle
        2. Applied to the laser direction via the scan angle.
    */

    double gamma = m_laserBeamDivergence / 1000;    // milliradians to radians

    MatrixXd c = MatrixXd::Zero(9, 9);

    // 1. lidar distance
    if (m_includeIncidenceAngle)
    {
        c(0, 0) = pow(m_sLidarDistance, 2.0)
                  + pow((lidarDist * tan(incidenceAngle) * gamma/4), 2.0);
    }
    else
    {
        c(0, 0) = pow(m_sLidarDistance, 2.0);
    }
    // 2. scan angle (left-right)
    c(1, 1) = pow(m_sScanAngle, 2.0) + pow(gamma/4, 2.0);
    // 3. scan angle (forward-back)
    c(2, 2) = pow(gamma/4, 2.0);
    // 4. sensor x
    c(3, 3) = pow(m_sSensorXY, 2.0);
    // 5. sensor y
    c(4, 4) = pow(m_sSensorXY, 2.0);
    // 6. sensor x
    c(5, 5) = pow(m_sSensorZ, 2.0);
    // 7. sensor roll
    c(6, 6) = pow(m_sSensorRollPitch, 2.0);
    // 8. sensor pitch
    c(7, 7) = pow(m_sSensorRollPitch, 2.0);
    // 9. sensor yaw
    c(8, 8) = pow(m_sSensorYaw, 2.0);

    return c;
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

    // laser to ground surface incidence angle (degrees)
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

    // scan angle (degrees)
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
