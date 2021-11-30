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
#include <nlohmann/json.hpp>
#include <pdal/io/LasWriter.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/util/FileUtils.hpp>


namespace pdal
{

    using namespace Dimension;
    using namespace Eigen;

    static PluginInfo const s_info
    {
        "filters.als_tpu",
        "Per-point airborne lidar Total Propagated Uncertainty (TPU) via a generic sensor model.",
        "https://github.com/pjhartzell/pdal-als-tpu"
    };

    CREATE_SHARED_STAGE(ALS_TPU, s_info)

    std::string ALS_TPU::getName() const
    {
        return s_info.name;
    }


    void ALS_TPU::addArgs(ProgramArgs& args)
    {
        args.add("uncertainty_file", "JSON file containing measurement standard deviations (uncertainties)", m_uncertaintyFile).setPositional();
        args.add("include_inc_angle", "Include incidence angle in TPU computation", m_includeIncidenceAngle, true);
        args.add("max_inc_angle", "Maximum allowable incidence angle (degrees <90)", m_maximumIncidenceAngle, 85.0);
        args.add("no_data_value", "TPU values when trajectory information is not available", m_noDataValue, -1.0);
        args.add("extended_output", "Export inverted observations, interpolated trajectory, and coordinate standard deviations", m_extendedOutput, false);
    }


    void ALS_TPU::initialize()
    {
        // check file existence
        if (!FileUtils::fileExists(m_uncertaintyFile))
            throw pdal_error("Cannot read measurement uncertainty file '" + m_uncertaintyFile + "'.");

        // open and check for an uncertainties entry
        std::istream* paramFile = FileUtils::openFile(m_uncertaintyFile);
        nlohmann::json params;
        *paramFile >> params;

        if (!params.contains("uncertainties"))
            throw pdal_error("No 'uncertainties' entry found in '" + m_uncertaintyFile + "'.");

        // pull out the uncertainties
        for (auto& entry : params["uncertainties"])
        {
            std::string name = entry["name"];
            double value = entry["value"];

            if (Utils::iequals(name, "std_lidar_range"))
                m_stdLidarRange = value;
            else if (Utils::iequals(name, "std_scan_angle"))
                m_stdScanAngle = value;
            else if (Utils::iequals(name, "std_sensor_xy"))
                m_stdSensorXy = value;
            else if (Utils::iequals(name, "std_sensor_z"))
                m_stdSensorZ = value;
            else if (Utils::iequals(name, "std_sensor_rollpitch"))
                m_stdSensorRollPitch = value;
            else if (Utils::iequals(name, "std_sensor_yaw"))
                m_stdSensorYaw = value;
            else if (Utils::iequals(name, "std_bore_rollpitch"))
                m_stdBoreRollPitch = value;
            else if (Utils::iequals(name, "std_bore_yaw"))
                m_stdBoreYaw = value;
            else if (Utils::iequals(name, "std_lever_xyz"))
                m_stdLeverXyz = value;
            else if (Utils::iequals(name, "beam_divergence"))
                m_beamDivergence = value;
            else
                throw pdal_error("Unrecognized uncertainty name '" + name + "' in '" + m_uncertaintyFile + "'.");
        }

        // standardize units
        m_stdScanAngle *= (M_PI / 180.0);           // degrees to radians
        m_stdSensorRollPitch *= (M_PI / 180.0);     // degrees to radians
        m_stdSensorYaw *= (M_PI / 180.0);           // degrees to radians
        m_stdBoreRollPitch *= (M_PI / 180.0);       // degrees to radians
        m_stdBoreYaw *= (M_PI / 180.0);             // degrees to radians
        m_maximumIncidenceAngle *= (M_PI / 180.0);  // degrees to radians
        m_beamDivergence /= 1000;                   // milliradians to radians
    }


    void ALS_TPU::addDimensions(PointLayoutPtr layout)
    {
        // standard output
        m_xVar = layout->registerOrAssignDim("VarianceX", Type::Float);
        m_yVar = layout->registerOrAssignDim("VarianceY", Type::Float);
        m_zVar = layout->registerOrAssignDim("VarianceZ", Type::Float);
        m_xyCov = layout->registerOrAssignDim("CovarianceXY", Type::Float);
        m_xzCov = layout->registerOrAssignDim("CovarianceXZ", Type::Float);
        m_yzCov = layout->registerOrAssignDim("CovarianceYZ", Type::Float);
        if (m_includeIncidenceAngle)
            m_incAngle = layout->registerOrAssignDim("IncidenceAngle", Type::Float);

        // extended output
        if (m_extendedOutput)
        {
            m_lidarRange = layout->registerOrAssignDim("LidarRange", Type::Float);
            m_scanAngleRL = layout->registerOrAssignDim("ScanAngleRL", Type::Float);
            m_scanAngleFB = layout->registerOrAssignDim("ScanAngleFB", Type::Float);

            m_xStd = layout->registerOrAssignDim("StdX", Type::Float);
            m_yStd = layout->registerOrAssignDim("StdY", Type::Float);
            m_zStd = layout->registerOrAssignDim("StdZ", Type::Float);

            m_trajX = layout->registerOrAssignDim("TrajX", Type::Float);
            m_trajY = layout->registerOrAssignDim("TrajY", Type::Float);
            m_trajZ = layout->registerOrAssignDim("TrajZ", Type::Float);
            m_trajRoll = layout->registerOrAssignDim("TrajRoll", Type::Float);
            m_trajPitch = layout->registerOrAssignDim("TrajPitch", Type::Float);
            m_trajHeading = layout->registerOrAssignDim("TrajHeading", Type::Float);
        }
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
            // 1. interpolate sensor xyz and attitude from trajectory points
            double trajX, trajY, trajZ, trajHeading, trajPitch;
            bool successfulInterp = linearInterpolation(
                cloud->getFieldAs<double>(Id::GpsTime, i),
                trajX, trajY, trajZ, trajHeading, trajPitch,
                trajectory, interpIdx);

            if (!successfulInterp)
            {
                cloud->setField(m_xVar, i, m_noDataValue);
                cloud->setField(m_yVar, i, m_noDataValue);
                cloud->setField(m_zVar, i, m_noDataValue);
                cloud->setField(m_xyCov, i, m_noDataValue);
                cloud->setField(m_xzCov, i, m_noDataValue);
                cloud->setField(m_yzCov, i, m_noDataValue);
                if (m_includeIncidenceAngle)
                    cloud->setField(m_incAngle, i, m_noDataValue);
            }
            else
            {
                // 2. invert for lidar distance and laser scan angle (left-right); estimate incidence angle
                double lidarDist, scanAngleRL, scanAngleFB, incidenceAngle;
                invertObservations(
                    cloud->point(i),
                    trajX, trajY, trajZ, trajHeading, trajPitch,
                    lidarDist, scanAngleRL, scanAngleFB, incidenceAngle);

                // 3. zero out observations that we do not estimate
                double trajRoll = 0.0;
                double boreRoll = 0.0;
                double borePitch = 0.0;
                double boreYaw = 0.0;
                double leverX = 0.0;
                double leverY = 0.0;
                double leverZ = 0.0;

                // 4. build observation covariance matrix
                MatrixXd obsCovariance = observationCovariance(
                    lidarDist,
                    incidenceAngle);

                // 5. propagate observation variance into point xyz covariance matrix
                Matrix3d lidarPointCovariance = propagateCovariance(
                    lidarDist, scanAngleRL, scanAngleFB,
                    trajX, trajY, trajZ,
                    trajRoll, trajPitch, trajHeading,
                    boreRoll, borePitch, boreYaw,
                    leverX, leverY, leverZ,
                    obsCovariance);

                // 6. standard output
                cloud->setField(m_xVar, i, lidarPointCovariance(0, 0));
                cloud->setField(m_yVar, i, lidarPointCovariance(1, 1));
                cloud->setField(m_zVar, i, lidarPointCovariance(2, 2));
                cloud->setField(m_xyCov, i, lidarPointCovariance(0, 1));
                cloud->setField(m_xzCov, i, lidarPointCovariance(0, 2));
                cloud->setField(m_yzCov, i, lidarPointCovariance(1, 2));
                if (m_includeIncidenceAngle)
                    cloud->setField(m_incAngle, i, incidenceAngle * 180.0 / M_PI);

                // 7. extended output
                if (m_extendedOutput)
                {
                    cloud->setField(m_lidarRange, i, lidarDist);
                    cloud->setField(m_scanAngleRL, i, scanAngleRL * 180.0 / M_PI);
                    cloud->setField(m_scanAngleFB, i, scanAngleFB * 180.0 / M_PI);

                    cloud->setField(m_xStd, i, sqrt(lidarPointCovariance(0, 0)));
                    cloud->setField(m_yStd, i, sqrt(lidarPointCovariance(1, 1)));
                    cloud->setField(m_zStd, i, sqrt(lidarPointCovariance(2, 2)));

                    cloud->setField(m_trajX, i, trajX);
                    cloud->setField(m_trajY, i, trajY);
                    cloud->setField(m_trajZ, i, trajZ);
                    cloud->setField(m_trajRoll, i, trajRoll * 180.0 / M_PI);
                    cloud->setField(m_trajPitch, i, trajPitch * 180.0 / M_PI);
                    cloud->setField(m_trajHeading, i, trajHeading * 180.0 / M_PI);
                }
            }
        }

        return cloud;
    }


    Matrix3d ALS_TPU::propagateCovariance(
        double lidarDist, double scanAngleRL, double scanAngleFB,
        double trajX, double trajY, double trajZ,
        double trajRoll, double trajPitch, double trajHeading,
        double boreRoll, double borePitch, double boreYaw,
        double leverX, double leverY, double leverZ,
        MatrixXd obsCovariance)
    {
        // precompute
        double sinTrajRoll = sin(trajRoll);
        double cosTrajRoll = cos(trajRoll);
        double sinTrajPitch = sin(trajPitch);
        double cosTrajPitch = cos(trajPitch);
        double sinTrajHeading = sin(trajHeading);
        double cosTrajHeading = cos(trajHeading);
        double sinBoreRoll = sin(boreRoll);
        double cosBoreRoll = cos(boreRoll);
        double sinBorePitch = sin(borePitch);
        double cosBorePitch = cos(borePitch);
        double sinBoreYaw = sin(boreYaw);
        double cosBoreYaw = cos(boreYaw);
        double sinScanAngleRL = sin(scanAngleRL);
        double cosScanAngleRL = cos(scanAngleRL);
        double sinScanAngleFB = sin(scanAngleFB);
        double cosScanAngleFB = cos(scanAngleFB);

        // Jacobian matrix ordered the same as the observation covariance matrix
        MatrixXd a = MatrixXd::Zero(3, 15);
        // partials with respect to lidar point X
        a(0, 0) = (sinTrajPitch*sinTrajRoll*sinTrajHeading + cosTrajRoll*cosTrajHeading)*((sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB + (sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL + sinScanAngleFB*sinBoreYaw*cosBorePitch) + (sinTrajPitch*sinTrajHeading*cosTrajRoll - sinTrajRoll*cosTrajHeading)*(-sinBorePitch*sinScanAngleFB + sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB + cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL) + ((sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleRL*cosScanAngleFB + (sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*cosScanAngleFB*cosScanAngleRL + sinScanAngleFB*cosBorePitch*cosBoreYaw)*sinTrajHeading*cosTrajPitch;
        a(0, 1) = (sinTrajPitch*sinTrajRoll*sinTrajHeading + cosTrajRoll*cosTrajHeading)*(lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL - lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB) + (sinTrajPitch*sinTrajHeading*cosTrajRoll - sinTrajRoll*cosTrajHeading)*(lidarDist*sinBoreRoll*cosBorePitch*cosScanAngleFB*cosScanAngleRL - lidarDist*sinScanAngleRL*cosBorePitch*cosBoreRoll*cosScanAngleFB) + (lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*cosScanAngleFB*cosScanAngleRL - lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*sinScanAngleRL*cosScanAngleFB)*sinTrajHeading*cosTrajPitch;
        a(0, 2) = (sinTrajPitch*sinTrajRoll*sinTrajHeading + cosTrajRoll*cosTrajHeading)*(-lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleFB*sinScanAngleRL - lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*sinScanAngleFB*cosScanAngleRL + lidarDist*sinBoreYaw*cosBorePitch*cosScanAngleFB) + (sinTrajPitch*sinTrajHeading*cosTrajRoll - sinTrajRoll*cosTrajHeading)*(-lidarDist*sinBorePitch*cosScanAngleFB - lidarDist*sinBoreRoll*sinScanAngleFB*sinScanAngleRL*cosBorePitch - lidarDist*sinScanAngleFB*cosBorePitch*cosBoreRoll*cosScanAngleRL) + (-lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleFB*sinScanAngleRL - lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*sinScanAngleFB*cosScanAngleRL + lidarDist*cosBorePitch*cosScanAngleFB*cosBoreYaw)*sinTrajHeading*cosTrajPitch;
        a(0, 3) = 1;
        a(0, 4) = 0;
        a(0, 5) = 0;
        a(0, 6) = (-sinTrajPitch*sinTrajRoll*sinTrajHeading - cosTrajRoll*cosTrajHeading)*(-lidarDist*sinBorePitch*sinScanAngleFB + lidarDist*sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB + lidarDist*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL + leverZ) + (sinTrajPitch*sinTrajHeading*cosTrajRoll - sinTrajRoll*cosTrajHeading)*(lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*sinBoreYaw*cosBorePitch + leverY);
        a(0, 7) = (-lidarDist*sinBorePitch*sinScanAngleFB + lidarDist*sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB + lidarDist*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL + leverZ)*sinTrajHeading*cosTrajPitch*cosTrajRoll + (lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*sinBoreYaw*cosBorePitch + leverY)*sinTrajRoll*sinTrajHeading*cosTrajPitch - (lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*cosBorePitch*cosBoreYaw + leverX)*sinTrajPitch*sinTrajHeading;
        a(0, 8) = (sinTrajPitch*sinTrajRoll*cosTrajHeading - sinTrajHeading*cosTrajRoll)*(lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*sinBoreYaw*cosBorePitch + leverY) + (sinTrajPitch*cosTrajRoll*cosTrajHeading + sinTrajRoll*sinTrajHeading)*(-lidarDist*sinBorePitch*sinScanAngleFB + lidarDist*sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB + lidarDist*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL + leverZ) + (lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*cosBorePitch*cosBoreYaw + leverX)*cosTrajPitch*cosTrajHeading;
        a(0, 9) = (sinTrajPitch*sinTrajRoll*sinTrajHeading + cosTrajRoll*cosTrajHeading)*(lidarDist*(-sinBorePitch*sinBoreRoll*sinBoreYaw - cosBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB) + (sinTrajPitch*sinTrajHeading*cosTrajRoll - sinTrajRoll*cosTrajHeading)*(-lidarDist*sinBoreRoll*cosBorePitch*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleRL*cosBorePitch*cosBoreRoll*cosScanAngleFB) + (lidarDist*(-sinBorePitch*sinBoreRoll*cosBoreYaw + sinBoreYaw*cosBoreRoll)*cosScanAngleFB*cosScanAngleRL + lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*sinScanAngleRL*cosScanAngleFB)*sinTrajHeading*cosTrajPitch;
        a(0, 10) = (sinTrajPitch*sinTrajRoll*sinTrajHeading + cosTrajRoll*cosTrajHeading)*(-lidarDist*sinBorePitch*sinScanAngleFB*sinBoreYaw + lidarDist*sinBoreRoll*sinScanAngleRL*sinBoreYaw*cosBorePitch*cosScanAngleFB + lidarDist*sinBoreYaw*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL) + (sinTrajPitch*sinTrajHeading*cosTrajRoll - sinTrajRoll*cosTrajHeading)*(-lidarDist*sinBorePitch*sinBoreRoll*sinScanAngleRL*cosScanAngleFB - lidarDist*sinBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL - lidarDist*sinScanAngleFB*cosBorePitch) + (-lidarDist*sinBorePitch*sinScanAngleFB*cosBoreYaw + lidarDist*sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB*cosBoreYaw + lidarDist*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL*cosBoreYaw)*sinTrajHeading*cosTrajPitch;
        a(0, 11) = (sinTrajPitch*sinTrajRoll*sinTrajHeading + cosTrajRoll*cosTrajHeading)*(lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*cosBorePitch*cosBoreYaw) + (lidarDist*(-sinBorePitch*sinBoreRoll*sinBoreYaw - cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB + lidarDist*(-sinBorePitch*sinBoreYaw*cosBoreRoll + sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL - lidarDist*sinScanAngleFB*sinBoreYaw*cosBorePitch)*sinTrajHeading*cosTrajPitch;
        a(0, 12) = sinTrajHeading*cosTrajPitch;
        a(0, 13) = sinTrajPitch*sinTrajRoll*sinTrajHeading + cosTrajRoll*cosTrajHeading;
        a(0, 14) = sinTrajPitch*sinTrajHeading*cosTrajRoll - sinTrajRoll*cosTrajHeading;
        // partials with respect to lidar point Y
        a(1, 0) = (sinTrajPitch*sinTrajRoll*cosTrajHeading - sinTrajHeading*cosTrajRoll)*((sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB + (sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL + sinScanAngleFB*sinBoreYaw*cosBorePitch) + (sinTrajPitch*cosTrajRoll*cosTrajHeading + sinTrajRoll*sinTrajHeading)*(-sinBorePitch*sinScanAngleFB + sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB + cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL) + ((sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleRL*cosScanAngleFB + (sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*cosScanAngleFB*cosScanAngleRL + sinScanAngleFB*cosBorePitch*cosBoreYaw)*cosTrajPitch*cosTrajHeading;
        a(1, 1) = (sinTrajPitch*sinTrajRoll*cosTrajHeading - sinTrajHeading*cosTrajRoll)*(lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL - lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB) + (sinTrajPitch*cosTrajRoll*cosTrajHeading + sinTrajRoll*sinTrajHeading)*(lidarDist*sinBoreRoll*cosBorePitch*cosScanAngleFB*cosScanAngleRL - lidarDist*sinScanAngleRL*cosBorePitch*cosBoreRoll*cosScanAngleFB) + (lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*cosScanAngleFB*cosScanAngleRL - lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*sinScanAngleRL*cosScanAngleFB)*cosTrajPitch*cosTrajHeading;
        a(1, 2) = (sinTrajPitch*sinTrajRoll*cosTrajHeading - sinTrajHeading*cosTrajRoll)*(-lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleFB*sinScanAngleRL - lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*sinScanAngleFB*cosScanAngleRL + lidarDist*sinBoreYaw*cosBorePitch*cosScanAngleFB) + (sinTrajPitch*cosTrajRoll*cosTrajHeading + sinTrajRoll*sinTrajHeading)*(-lidarDist*sinBorePitch*cosScanAngleFB - lidarDist*sinBoreRoll*sinScanAngleFB*sinScanAngleRL*cosBorePitch - lidarDist*sinScanAngleFB*cosBorePitch*cosBoreRoll*cosScanAngleRL) + (-lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleFB*sinScanAngleRL - lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*sinScanAngleFB*cosScanAngleRL + lidarDist*cosBorePitch*cosScanAngleFB*cosBoreYaw)*cosTrajPitch*cosTrajHeading;
        a(1, 3) = 0;
        a(1, 4) = 1;
        a(1, 5) = 0;
        a(1, 6) = (-sinTrajPitch*sinTrajRoll*cosTrajHeading + sinTrajHeading*cosTrajRoll)*(-lidarDist*sinBorePitch*sinScanAngleFB + lidarDist*sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB + lidarDist*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL + leverZ) + (sinTrajPitch*cosTrajRoll*cosTrajHeading + sinTrajRoll*sinTrajHeading)*(lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*sinBoreYaw*cosBorePitch + leverY);
        a(1, 7) = (-lidarDist*sinBorePitch*sinScanAngleFB + lidarDist*sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB + lidarDist*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL + leverZ)*cosTrajPitch*cosTrajRoll*cosTrajHeading + (lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*sinBoreYaw*cosBorePitch + leverY)*sinTrajRoll*cosTrajPitch*cosTrajHeading - (lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*cosBorePitch*cosBoreYaw + leverX)*sinTrajPitch*cosTrajHeading;
        a(1, 8) = (-sinTrajPitch*sinTrajRoll*sinTrajHeading - cosTrajRoll*cosTrajHeading)*(lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*sinBoreYaw*cosBorePitch + leverY) + (-sinTrajPitch*sinTrajHeading*cosTrajRoll + sinTrajRoll*cosTrajHeading)*(-lidarDist*sinBorePitch*sinScanAngleFB + lidarDist*sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB + lidarDist*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL + leverZ) - (lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*cosBorePitch*cosBoreYaw + leverX)*sinTrajHeading*cosTrajPitch;
        a(1, 9) = (sinTrajPitch*sinTrajRoll*cosTrajHeading - sinTrajHeading*cosTrajRoll)*(lidarDist*(-sinBorePitch*sinBoreRoll*sinBoreYaw - cosBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB) + (sinTrajPitch*cosTrajRoll*cosTrajHeading + sinTrajRoll*sinTrajHeading)*(-lidarDist*sinBoreRoll*cosBorePitch*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleRL*cosBorePitch*cosBoreRoll*cosScanAngleFB) + (lidarDist*(-sinBorePitch*sinBoreRoll*cosBoreYaw + sinBoreYaw*cosBoreRoll)*cosScanAngleFB*cosScanAngleRL + lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*sinScanAngleRL*cosScanAngleFB)*cosTrajPitch*cosTrajHeading;
        a(1, 10) = (sinTrajPitch*sinTrajRoll*cosTrajHeading - sinTrajHeading*cosTrajRoll)*(-lidarDist*sinBorePitch*sinScanAngleFB*sinBoreYaw + lidarDist*sinBoreRoll*sinScanAngleRL*sinBoreYaw*cosBorePitch*cosScanAngleFB + lidarDist*sinBoreYaw*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL) + (sinTrajPitch*cosTrajRoll*cosTrajHeading + sinTrajRoll*sinTrajHeading)*(-lidarDist*sinBorePitch*sinBoreRoll*sinScanAngleRL*cosScanAngleFB - lidarDist*sinBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL - lidarDist*sinScanAngleFB*cosBorePitch) + (-lidarDist*sinBorePitch*sinScanAngleFB*cosBoreYaw + lidarDist*sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB*cosBoreYaw + lidarDist*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL*cosBoreYaw)*cosTrajPitch*cosTrajHeading;
        a(1, 11) = (sinTrajPitch*sinTrajRoll*cosTrajHeading - sinTrajHeading*cosTrajRoll)*(lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*cosBorePitch*cosBoreYaw) + (lidarDist*(-sinBorePitch*sinBoreRoll*sinBoreYaw - cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB + lidarDist*(-sinBorePitch*sinBoreYaw*cosBoreRoll + sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL - lidarDist*sinScanAngleFB*sinBoreYaw*cosBorePitch)*cosTrajPitch*cosTrajHeading;
        a(1, 12) = cosTrajPitch*cosTrajHeading;
        a(1, 13) = sinTrajPitch*sinTrajRoll*cosTrajHeading - sinTrajHeading*cosTrajRoll;
        a(1, 14) = sinTrajPitch*cosTrajRoll*cosTrajHeading + sinTrajRoll*sinTrajHeading;
        // partials with respect to lidar point Z
        a(2, 0) = (sinBorePitch*sinScanAngleFB - sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB - cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL)*cosTrajPitch*cosTrajRoll + (-(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB - (sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL - sinScanAngleFB*sinBoreYaw*cosBorePitch)*sinTrajRoll*cosTrajPitch + ((sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleRL*cosScanAngleFB + (sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*cosScanAngleFB*cosScanAngleRL + sinScanAngleFB*cosBorePitch*cosBoreYaw)*sinTrajPitch;
        a(2, 1) = (-lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB)*sinTrajRoll*cosTrajPitch + (lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*cosScanAngleFB*cosScanAngleRL - lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*sinScanAngleRL*cosScanAngleFB)*sinTrajPitch + (-lidarDist*sinBoreRoll*cosBorePitch*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleRL*cosBorePitch*cosBoreRoll*cosScanAngleFB)*cosTrajPitch*cosTrajRoll;
        a(2, 2) = (lidarDist*sinBorePitch*cosScanAngleFB + lidarDist*sinBoreRoll*sinScanAngleFB*sinScanAngleRL*cosBorePitch + lidarDist*sinScanAngleFB*cosBorePitch*cosBoreRoll*cosScanAngleRL)*cosTrajPitch*cosTrajRoll + (lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleFB*sinScanAngleRL + lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*sinScanAngleFB*cosScanAngleRL - lidarDist*sinBoreYaw*cosBorePitch*cosScanAngleFB)*sinTrajRoll*cosTrajPitch + (-lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleFB*sinScanAngleRL - lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*sinScanAngleFB*cosScanAngleRL + lidarDist*cosBorePitch*cosScanAngleFB*cosBoreYaw)*sinTrajPitch;
        a(2, 3) = 0;
        a(2, 4) = 0;
        a(2, 5) = 1;
        a(2, 6) = -(lidarDist*sinBorePitch*sinScanAngleFB - lidarDist*sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB - lidarDist*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL - leverZ)*sinTrajRoll*cosTrajPitch + (-lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB - lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL - lidarDist*sinScanAngleFB*sinBoreYaw*cosBorePitch - leverY)*cosTrajPitch*cosTrajRoll;
        a(2, 7) = -(lidarDist*sinBorePitch*sinScanAngleFB - lidarDist*sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB - lidarDist*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL - leverZ)*sinTrajPitch*cosTrajRoll - (-lidarDist*(sinBorePitch*sinBoreRoll*sinBoreYaw + cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB - lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL - lidarDist*sinScanAngleFB*sinBoreYaw*cosBorePitch - leverY)*sinTrajPitch*sinTrajRoll + (lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleRL*cosScanAngleFB + lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*cosBorePitch*cosBoreYaw + leverX)*cosTrajPitch;
        a(2, 8) = 0;
        a(2, 9) = (-lidarDist*(-sinBorePitch*sinBoreRoll*sinBoreYaw - cosBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL - lidarDist*(sinBorePitch*sinBoreYaw*cosBoreRoll - sinBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB)*sinTrajRoll*cosTrajPitch + (lidarDist*(-sinBorePitch*sinBoreRoll*cosBoreYaw + sinBoreYaw*cosBoreRoll)*cosScanAngleFB*cosScanAngleRL + lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*sinScanAngleRL*cosScanAngleFB)*sinTrajPitch + (lidarDist*sinBoreRoll*cosBorePitch*cosScanAngleFB*cosScanAngleRL - lidarDist*sinScanAngleRL*cosBorePitch*cosBoreRoll*cosScanAngleFB)*cosTrajPitch*cosTrajRoll;
        a(2, 10) = (lidarDist*sinBorePitch*sinScanAngleFB*sinBoreYaw - lidarDist*sinBoreRoll*sinScanAngleRL*sinBoreYaw*cosBorePitch*cosScanAngleFB - lidarDist*sinBoreYaw*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL)*sinTrajRoll*cosTrajPitch + (-lidarDist*sinBorePitch*sinScanAngleFB*cosBoreYaw + lidarDist*sinBoreRoll*sinScanAngleRL*cosBorePitch*cosScanAngleFB*cosBoreYaw + lidarDist*cosBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL*cosBoreYaw)*sinTrajPitch + (lidarDist*sinBorePitch*sinBoreRoll*sinScanAngleRL*cosScanAngleFB + lidarDist*sinBorePitch*cosBoreRoll*cosScanAngleFB*cosScanAngleRL + lidarDist*sinScanAngleFB*cosBorePitch)*cosTrajPitch*cosTrajRoll;
        a(2, 11) = (lidarDist*(-sinBorePitch*sinBoreRoll*sinBoreYaw - cosBoreRoll*cosBoreYaw)*sinScanAngleRL*cosScanAngleFB + lidarDist*(-sinBorePitch*sinBoreYaw*cosBoreRoll + sinBoreRoll*cosBoreYaw)*cosScanAngleFB*cosScanAngleRL - lidarDist*sinScanAngleFB*sinBoreYaw*cosBorePitch)*sinTrajPitch + (-lidarDist*(sinBorePitch*sinBoreRoll*cosBoreYaw - sinBoreYaw*cosBoreRoll)*sinScanAngleRL*cosScanAngleFB - lidarDist*(sinBorePitch*cosBoreRoll*cosBoreYaw + sinBoreRoll*sinBoreYaw)*cosScanAngleFB*cosScanAngleRL - lidarDist*sinScanAngleFB*cosBorePitch*cosBoreYaw)*sinTrajRoll*cosTrajPitch;
        a(2, 12) = sinTrajPitch;
        a(2, 13) = -sinTrajRoll*cosTrajPitch;
        a(2, 14) = -cosTrajPitch*cosTrajRoll;

        // propagate
        Matrix3d c = a * obsCovariance * a.transpose();

        return c;
    }



    MatrixXd ALS_TPU::observationCovariance(
        double lidarDist, double incidenceAngle)
    {
        /*
        Variance order: 
            1. lidar distance
            2. scan angle (right-left)
            3. scan angle (forward-back)
            4. sensor x
            5. sensor y
            6. sensor z
            7. sensor roll
            8. sensor pitch
            9. sensor heading
            10. boresight roll
            11. boresight pitch
            12. boresight yaw
            13. lever x
            14. lever y
            15. lever z

        Beam divergence uncertainty:
            1. Applied to the lidar distance when including the incidence angle
            2. Applied to the laser direction via the scan angle. Note that beam
               divergence is the reason we have a forward/back scan angle, which
               allows us to add the beam uncertainty in both the left/right and 
               forward/back directions
        */

        MatrixXd c = MatrixXd::Zero(15, 15);

        // 1. lidar distance
        if (m_includeIncidenceAngle)
        {
            c(0, 0) = pow(m_stdLidarRange, 2.0)
                    + pow((lidarDist * tan(incidenceAngle) * m_beamDivergence/4), 2.0);
        }
        else
        {
            c(0, 0) = pow(m_stdLidarRange, 2.0);
        }
        // 2. scan angle (right-left)
        c(1, 1) = pow(m_stdScanAngle, 2.0) + pow(m_beamDivergence/4, 2.0);
        // 3. scan angle (forward-back)
        c(2, 2) = pow(m_beamDivergence/4, 2.0);
        // 4. sensor x
        c(3, 3) = pow(m_stdSensorXy, 2.0);
        // 5. sensor y
        c(4, 4) = pow(m_stdSensorXy, 2.0);
        // 6. sensor x
        c(5, 5) = pow(m_stdSensorZ, 2.0);
        // 7. sensor roll
        c(6, 6) = pow(m_stdSensorRollPitch, 2.0);
        // 8. sensor pitch
        c(7, 7) = pow(m_stdSensorRollPitch, 2.0);
        // 9. sensor yaw
        c(8, 8) = pow(m_stdSensorYaw, 2.0);
        // 10. boresight roll
        c(9, 9) = pow(m_stdBoreRollPitch, 2.0);
        // 11. boresight pitch
        c(10, 10) = pow(m_stdBoreRollPitch, 2.0);
        // 12. boresight yaw
        c(11, 11) = pow(m_stdBoreYaw, 2.0);
        // 13. lever x
        c(12, 12) = pow(m_stdLeverXyz, 2.0);
        // 14. lever y
        c(13, 13) = pow(m_stdLeverXyz, 2.0);
        // 15. lever z
        c(14, 14) = pow(m_stdLeverXyz, 2.0);

        return c;
    }


    void ALS_TPU::invertObservations(
        PointRef cloudPoint,
        double trajX, double trajY, double trajZ,
        double trajHeading, double trajPitch,
        double& lidarDist, double& scanAngleRL, double& scanAngleFB,
        double& incidenceAngle)
    {
        // lidar range
        Vector3d laserVector;
        laserVector << (cloudPoint.getFieldAs<double>(Id::X) - trajX),
                       (cloudPoint.getFieldAs<double>(Id::Y) - trajY),
                       (cloudPoint.getFieldAs<double>(Id::Z) - trajZ);
        lidarDist = laserVector.norm();

        // incidence: angle between laser ray and ground normal vector
        if (m_includeIncidenceAngle)
        {
            Vector3d groundNormalVector, unitLaserVector;
            groundNormalVector << cloudPoint.getFieldAs<double>(Id::NormalX),
                                cloudPoint.getFieldAs<double>(Id::NormalY),
                                cloudPoint.getFieldAs<double>(Id::NormalZ);
            unitLaserVector = laserVector / lidarDist;
            incidenceAngle = acos(unitLaserVector.dot(-groundNormalVector));
            if (incidenceAngle > m_maximumIncidenceAngle)
                incidenceAngle = m_maximumIncidenceAngle;
        }
        else
            incidenceAngle = 0.0;

        // inverse of IMU frame to NED frame
        Matrix3d inverseImu;
        inverseImu << cos(trajPitch)*cos(trajHeading), sin(trajHeading)*cos(trajPitch), -sin(trajPitch),
                     -sin(trajHeading), cos(trajHeading), 0,
                      sin(trajPitch)*cos(trajHeading), sin(trajPitch)*sin(trajHeading), cos(trajPitch);
        // inverse of NED frame to ENU frame
        Matrix3d inverseCOB;
        inverseCOB << 0, 1, 0,
                      1, 0, 0,
                      0, 0, -1;

        // transform laser vector from world frame to scanner frame
        laserVector = inverseImu * inverseCOB * laserVector;

        // forward-back (FB) and right-left (RL) scan angles
        scanAngleFB = asin(laserVector(0) / lidarDist);
        scanAngleRL = asin(laserVector(1) / (lidarDist*cos(scanAngleFB)));
    }


    bool ALS_TPU::linearInterpolation(
        double pointTime,
        double& trajX, double& trajY, double& trajZ,
        double& trajHeading, double& trajPitch,
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

        trajHeading = (trajectory->getFieldAs<double>(Id::Azimuth, interpIdx)
        + factor * (trajectory->getFieldAs<double>(Id::Azimuth, interpIdx + 1)
        - trajectory->getFieldAs<double>(Id::Azimuth, interpIdx)))
        * (M_PI / 180.0);

        trajPitch = (trajectory->getFieldAs<double>(Id::Pitch, interpIdx)
        + factor * (trajectory->getFieldAs<double>(Id::Pitch, interpIdx + 1)
        - trajectory->getFieldAs<double>(Id::Pitch, interpIdx)))
        * (M_PI / 180.0);

        return true;
    }

} // namespace pdal
