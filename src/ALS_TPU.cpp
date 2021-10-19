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
        args.add("profile", "Predefined or custom sensor observation uncertainties", m_paramProfile).setPositional();

        args.add("s_lidar_distance", "Lidar distance standard deviation (meters)", m_sLidarDistance, 0.0);
        args.add("s_scan_angle", "Scan angle standard deviation (degrees)", m_sScanAngle, 0.0);
        args.add("s_sensor_xy", "Sensor x and y position standard deviation (meters)", m_sSensorXY, 0.0);
        args.add("s_sensor_z", "Sensor z position standard deviation (meters)", m_sSensorZ, 0.0);
        args.add("s_sensor_rollpitch", "Sensor roll and pitch standard deviation (degrees)", m_sSensorRollPitch, 0.0);
        args.add("s_sensor_yaw", "Sensor yaw standard deviation (degrees)", m_sSensorYaw, 0.0);
        args.add("s_bore_rollpitch", "Boresight roll and pitch standard deviation (degrees)", m_sBoreRollPitch, 0.0);
        args.add("s_bore_yaw", "Boresight yaw standard deviation (degrees)", m_sBoreYaw, 0.0);
        args.add("s_lever_x", "IMU to scanner lever x component standard deviation (meters)", m_sLeverX, 0.0);
        args.add("s_lever_y", "IMU to scanner lever y component standard deviation (meters)", m_sLeverY, 0.0);
        args.add("s_lever_z", "IMU to scanner lever z component standard deviation (meters)", m_sLeverZ, 0.0);
        args.add("laser_beam_divergence", "Laser beam divergence, 1/e^2 definition (milliradians)", m_laserBeamDivergence, 0.0);

        args.add("include_inc_angle", "Include incidence angle in TPU computation", m_includeIncidenceAngle, true);
        args.add("max_inc_angle", "Maximum allowable incidence angle (degrees <90)", m_maximumIncidenceAngle, 85.0);

        args.add("no_data_value", "TPU values when trajectory information is not available", m_noDataValue, -1.0);
    }


    void ALS_TPU::initialize()
    {
        // set parameters
        setParams();

        // degrees to radians
        m_sScanAngle *= (M_PI / 180.0);
        m_sSensorRollPitch *= (M_PI / 180.0);
        m_sSensorYaw *= (M_PI / 180.0);
        m_sBoreRollPitch *= (M_PI / 180.0);
        m_sBoreYaw *= (M_PI / 180.0);
        m_maximumIncidenceAngle *= (M_PI / 180.0);

        // milliradians to radians
        m_laserBeamDivergence /= 1000;

        // valid input
        if (m_includeIncidenceAngle && (m_maximumIncidenceAngle == 0.0))
        {
            throwError("Maximum incidence angle must be greater than 0 degrees.");
        }
        if (!Utils::iequals(m_paramProfile, "custom") && !Utils::iequals(m_paramProfile, "titan-channel2"))
        {
            throwError("Invalid parameter profile designation.");
        }
    }


    void ALS_TPU::addDimensions(PointLayoutPtr layout)
    {
        m_lidarDist = layout->registerOrAssignDim("LidarDistance", Type::Double);
        m_scanAngleRL = layout->registerOrAssignDim("ScanAngleRL", Type::Double);
        m_scanAngleFB = layout->registerOrAssignDim("ScanAngleFB", Type::Double);
        m_incAngle = layout->registerOrAssignDim("IncAngle", Type::Double);

        m_xVar = layout->registerOrAssignDim("VarianceX", Type::Double);
        m_yVar = layout->registerOrAssignDim("VarianceY", Type::Double);
        m_zVar = layout->registerOrAssignDim("VarianceZ", Type::Double);
        m_xyCov = layout->registerOrAssignDim("CovarianceXY", Type::Double);
        m_xzCov = layout->registerOrAssignDim("CovarianceXZ", Type::Double);
        m_yzCov = layout->registerOrAssignDim("CovarianceYZ", Type::Double);
        
        m_xStd = layout->registerOrAssignDim("StdX", Type::Double);
        m_yStd = layout->registerOrAssignDim("StdY", Type::Double);
        m_zStd = layout->registerOrAssignDim("StdZ", Type::Double);

        m_trajX = layout->registerOrAssignDim("TrajX", Type::Double);
        m_trajY = layout->registerOrAssignDim("TrajY", Type::Double);
        m_trajZ = layout->registerOrAssignDim("TrajZ", Type::Double);
        m_trajRoll = layout->registerOrAssignDim("TrajRoll", Type::Double);
        m_trajPitch = layout->registerOrAssignDim("TrajPitch", Type::Double);
        m_trajHeading = layout->registerOrAssignDim("TrajHeading", Type::Double);
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


    void ALS_TPU::setParams()
    {
        if (Utils::iequals(m_paramProfile, "custom"))
        {
            // Do nothing. The parameter values are set by the user.
        }
        else if (Utils::iequals(m_paramProfile, "titan-channel2"))
        {
            // Optech Titan with a Northrup Grumman LN200
            // meters (datasheet)
            m_sLidarDistance = 0.008;
            // degrees (unknown, using angular resolution of 0.001 deg from
            // Optech 3100 shown in Glennie's 2007 JAG paper)
            m_sScanAngle = 0.001 / sqrt(12);
            // meters (datasheet, airborne PP RMS = 1 cm)
            m_sSensorXY = 0.01;
            // meters (datasheet, airborne PP RMS = 2 cm)
            m_sSensorZ = 0.02;
            // degrees (datasheet, PP RMS = 0.005 degrees)
            m_sSensorRollPitch = 0.005;
            // degrees (datasheet, PP RMS = 0.007 degrees)
            m_sSensorYaw = 0.007;
            // degrees (Glennie, 2007, JAG, Table 2)
            m_sBoreRollPitch = 0.001;
            // degrees (Glennie, 2007, JAG, Table 2)
            m_sBoreYaw = 0.004;
            // meters (conservative estimate from Glennie's 2007 JAG paper)
            m_sLeverX = 0.02;
            m_sLeverY = 0.02;
            m_sLeverZ = 0.02;
            // milliradians (datasheet, 0.35 mrad at 1/e for channel #2)
            m_laserBeamDivergence = 0.35 * sqrt(2);
        }
        // else if (Utils::iequals(m_paramProfile, "sitka"))
        // {
        //     // Riegl VQ-480i with an ATLANS C IMU
        //     // meters (datasheet)
        //     m_sLidarDistance = 0.008;
        //     // degrees (unknown, using angular resolution of 0.001 deg from
        //     // Optech 3100 shown in Glennie's 2007 JAG paper)
        //     m_sScanAngle = 0.001 / sqrt(12);
        //     // meters (datasheet, airborne PP RMS = 1 cm)
        //     m_sSensorXY = 0.01;
        //     // meters (datasheet, airborne PP RMS = 2 cm)
        //     m_sSensorZ = 0.02;
        //     // degrees (datasheet, PP RMS = 0.005 degrees)
        //     m_sSensorRollPitch = 0.005;
        //     // degrees (datasheet, PP RMS = 0.007 degrees)
        //     m_sSensorYaw = 0.007;
        //     // degrees (Glennie, 2007, JAG, Table 2)
        //     m_sBoreRollPitch = 0.001;
        //     // degrees (Glennie, 2007, JAG, Table 2)
        //     m_sBoreYaw = 0.004;
        //     // meters (unknown, using conservative estimate from Glennie's 2007
        //     // JAG paper)
        //     m_sLeverX = 0.02;
        //     m_sLeverY = 0.02;
        //     m_sLeverZ = 0.02;
        //     // milliradians (datasheet, 0.35 mrad at 1/e for channel #2)
        //     m_laserBeamDivergence = 0.35 * sqrt(2);
        // }
    }


    PointViewPtr ALS_TPU::tpu(PointViewPtr cloud, PointViewPtr trajectory)
    {

        PointId interpIdx = 0;

        // iterate over each cloud point
        for (PointId i = 0; i < cloud->size(); ++i)
        {
            // 1. interpolate sensor xyz and heading from trajectory points
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
            }
            else
            {
                // 2. invert for lidar distance and laser scan angle (left-right); estimate incidence angle
                double lidarDist, scanAngleRL, scanAngleFB, incidenceAngle;
                invertObservations(
                    cloud->point(i),
                    trajX, trajY, trajZ, trajHeading, trajPitch,
                    lidarDist, scanAngleRL, scanAngleFB, incidenceAngle);

                // 3. build observation covariance matrix
                MatrixXd obsCovariance = observationCovariance(
                    lidarDist,
                    incidenceAngle);

                // 4. zero out items that we do not estimate (roll, pitch, forward/back scan angle)
                double trajRoll = 0.0;
                double boreRoll = 0.0;
                double borePitch = 0.0;
                double boreYaw = 0.0;
                double leverX = 0.0;
                double leverY = 0.0;
                double leverZ = 0.0;

                // 5. propagate observation variance into point xyz covariance matrix
                Matrix3d lidarPointCovariance = propagateCovariance(
                    lidarDist, scanAngleRL, scanAngleFB,
                    trajX, trajY, trajZ,
                    trajRoll, trajPitch, trajHeading,
                    boreRoll, borePitch, boreYaw,
                    leverX, leverY, leverZ,
                    obsCovariance);

                // 6. store covariance information in new dimensions
                cloud->setField(m_xVar, i, lidarPointCovariance(0, 0));
                cloud->setField(m_yVar, i, lidarPointCovariance(1, 1));
                cloud->setField(m_zVar, i, lidarPointCovariance(2, 2));
                cloud->setField(m_xyCov, i, lidarPointCovariance(0, 1));
                cloud->setField(m_xzCov, i, lidarPointCovariance(0, 2));
                cloud->setField(m_yzCov, i, lidarPointCovariance(1, 2));

                // Temporary test output
                cloud->setField(m_lidarDist, i, lidarDist);
                cloud->setField(m_scanAngleRL, i, scanAngleRL * 180.0 / M_PI);
                cloud->setField(m_scanAngleFB, i, scanAngleFB * 180.0 / M_PI);
                cloud->setField(m_incAngle, i, incidenceAngle * 180.0 / M_PI);

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

        return cloud;
    }


    Matrix3d ALS_TPU::propagateCovariance(
        double lidarDist, double scanAngleLR, double scanAngleFB,
        double trajX, double trajY, double trajZ,
        double trajRoll, double trajPitch, double trajHeading,
        double boreRoll, double borePitch, double boreYaw,
        double leverX, double leverY, double leverZ,
        MatrixXd obsCovariance)
    {
        double trajYaw = -trajHeading;  // heading to yaw

        // Jacobian matrix ordered the same as the observation covariance matrix
        MatrixXd a = MatrixXd::Zero(3, 15);
        // partials with respect to lidar point X
        a(0, 0) = (sin(trajPitch)*sin(trajRoll)*sin(trajYaw) + cos(trajRoll)*cos(trajYaw))*((sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) + (-sin(borePitch)*sin(boreYaw)*cos(boreRoll) + sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + sin(scanAngleFB)*sin(boreYaw)*cos(borePitch)) + (sin(trajPitch)*sin(trajYaw)*cos(trajRoll) - sin(trajRoll)*cos(trajYaw))*(-sin(borePitch)*sin(scanAngleFB) + sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB) - cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR)) + ((sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleLR)*cos(scanAngleFB) + (-sin(borePitch)*cos(boreRoll)*cos(boreYaw) - sin(boreRoll)*sin(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + sin(scanAngleFB)*cos(borePitch)*cos(boreYaw))*sin(trajYaw)*cos(trajPitch);
        a(0, 1) = (sin(trajPitch)*sin(trajRoll)*sin(trajYaw) + cos(trajRoll)*cos(trajYaw))*(lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB)) + (sin(trajPitch)*sin(trajYaw)*cos(trajRoll) - sin(trajRoll)*cos(trajYaw))*(lidarDist*sin(boreRoll)*cos(borePitch)*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleLR)*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)) + (lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB))*sin(trajYaw)*cos(trajPitch);
        a(0, 2) = (sin(trajPitch)*sin(trajRoll)*sin(trajYaw) + cos(trajRoll)*cos(trajYaw))*(-lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleFB)*sin(scanAngleLR) + lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*sin(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(boreYaw)*cos(borePitch)*cos(scanAngleFB)) + (sin(trajPitch)*sin(trajYaw)*cos(trajRoll) - sin(trajRoll)*cos(trajYaw))*(-lidarDist*sin(borePitch)*cos(scanAngleFB) - lidarDist*sin(boreRoll)*sin(scanAngleFB)*sin(scanAngleLR)*cos(borePitch) + lidarDist*sin(scanAngleFB)*cos(borePitch)*cos(boreRoll)*cos(scanAngleLR)) + (-lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleFB)*sin(scanAngleLR) + lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*sin(scanAngleFB)*cos(scanAngleLR) + lidarDist*cos(borePitch)*cos(scanAngleFB)*cos(boreYaw))*sin(trajYaw)*cos(trajPitch);
        a(0, 3) = 1;
        a(0, 4) = 0;
        a(0, 5) = 0;
        a(0, 6) = (-sin(trajPitch)*sin(trajRoll)*sin(trajYaw) - cos(trajRoll)*cos(trajYaw))*(-lidarDist*sin(borePitch)*sin(scanAngleFB) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB) - lidarDist*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR) + leverZ) + (sin(trajPitch)*sin(trajYaw)*cos(trajRoll) - sin(trajRoll)*cos(trajYaw))*(lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*sin(boreYaw)*cos(borePitch) + leverX);
        a(0, 7) = (-lidarDist*sin(borePitch)*sin(scanAngleFB) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB) - lidarDist*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR) + leverZ)*sin(trajYaw)*cos(trajPitch)*cos(trajRoll) + (lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*sin(boreYaw)*cos(borePitch) + leverX)*sin(trajRoll)*sin(trajYaw)*cos(trajPitch) - (lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*cos(borePitch)*cos(boreYaw) + leverY)*sin(trajPitch)*sin(trajYaw);
        a(0, 8) = (sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*(lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*sin(boreYaw)*cos(borePitch) + leverX) + (sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*(-lidarDist*sin(borePitch)*sin(scanAngleFB) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB) - lidarDist*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR) + leverZ) + (lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*cos(borePitch)*cos(boreYaw) + leverY)*cos(trajPitch)*cos(trajYaw);
        a(0, 9) = (sin(trajPitch)*sin(trajRoll)*sin(trajYaw) + cos(trajRoll)*cos(trajYaw))*(-lidarDist*(-sin(borePitch)*sin(boreRoll)*sin(boreYaw) - cos(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB)) + (sin(trajPitch)*sin(trajYaw)*cos(trajRoll) - sin(trajRoll)*cos(trajYaw))*(lidarDist*sin(boreRoll)*cos(borePitch)*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleLR)*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)) + (-lidarDist*(-sin(borePitch)*sin(boreRoll)*cos(boreYaw) + sin(boreYaw)*cos(boreRoll))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB))*sin(trajYaw)*cos(trajPitch);
        a(0, 10) = (sin(trajPitch)*sin(trajRoll)*sin(trajYaw) + cos(trajRoll)*cos(trajYaw))*(-lidarDist*sin(borePitch)*sin(scanAngleFB)*sin(boreYaw) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*sin(boreYaw)*cos(borePitch)*cos(scanAngleFB) - lidarDist*sin(boreYaw)*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR)) + (sin(trajPitch)*sin(trajYaw)*cos(trajRoll) - sin(trajRoll)*cos(trajYaw))*(-lidarDist*sin(borePitch)*sin(boreRoll)*sin(scanAngleLR)*cos(scanAngleFB) + lidarDist*sin(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*sin(scanAngleFB)*cos(borePitch)) + (-lidarDist*sin(borePitch)*sin(scanAngleFB)*cos(boreYaw) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB)*cos(boreYaw) - lidarDist*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR)*cos(boreYaw))*sin(trajYaw)*cos(trajPitch);
        a(0, 11) = (sin(trajPitch)*sin(trajRoll)*sin(trajYaw) + cos(trajRoll)*cos(trajYaw))*(lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*cos(borePitch)*cos(boreYaw)) + (lidarDist*(-sin(borePitch)*sin(boreRoll)*sin(boreYaw) - cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(-sin(borePitch)*sin(boreYaw)*cos(boreRoll) + sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*sin(scanAngleFB)*sin(boreYaw)*cos(borePitch))*sin(trajYaw)*cos(trajPitch);
        a(0, 12) = sin(trajPitch)*sin(trajRoll)*sin(trajYaw) + cos(trajRoll)*cos(trajYaw);
        a(0, 13) = (trajYaw)*cos(trajPitch);
        a(0, 14) = sin(trajPitch)*sin(trajYaw)*cos(trajRoll) - sin(trajRoll)*cos(trajYaw);
        // partials with respect to lidar point Y
        a(1, 0) = (sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*((sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) + (-sin(borePitch)*sin(boreYaw)*cos(boreRoll) + sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + sin(scanAngleFB)*sin(boreYaw)*cos(borePitch)) + (sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*(-sin(borePitch)*sin(scanAngleFB) + sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB) - cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR)) + ((sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleLR)*cos(scanAngleFB) + (-sin(borePitch)*cos(boreRoll)*cos(boreYaw) - sin(boreRoll)*sin(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + sin(scanAngleFB)*cos(borePitch)*cos(boreYaw))*cos(trajPitch)*cos(trajYaw);
        a(1, 1) = (sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*(lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB)) + (sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*(lidarDist*sin(boreRoll)*cos(borePitch)*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleLR)*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)) + (lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB))*cos(trajPitch)*cos(trajYaw);
        a(1, 2) = (sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*(-lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleFB)*sin(scanAngleLR) + lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*sin(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(boreYaw)*cos(borePitch)*cos(scanAngleFB)) + (sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*(-lidarDist*sin(borePitch)*cos(scanAngleFB) - lidarDist*sin(boreRoll)*sin(scanAngleFB)*sin(scanAngleLR)*cos(borePitch) + lidarDist*sin(scanAngleFB)*cos(borePitch)*cos(boreRoll)*cos(scanAngleLR)) + (-lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleFB)*sin(scanAngleLR) + lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*sin(scanAngleFB)*cos(scanAngleLR) + lidarDist*cos(borePitch)*cos(scanAngleFB)*cos(boreYaw))*cos(trajPitch)*cos(trajYaw);
        a(1, 3) = 0;
        a(1, 4) = 1;
        a(1, 5) = 0;
        a(1, 6) = (-sin(trajPitch)*sin(trajRoll)*cos(trajYaw) + sin(trajYaw)*cos(trajRoll))*(-lidarDist*sin(borePitch)*sin(scanAngleFB) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB) - lidarDist*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR) + leverZ) + (sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*(lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*sin(boreYaw)*cos(borePitch) + leverX);
        a(1, 7) = (-lidarDist*sin(borePitch)*sin(scanAngleFB) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB) - lidarDist*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR) + leverZ)*cos(trajPitch)*cos(trajRoll)*cos(trajYaw) + (lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*sin(boreYaw)*cos(borePitch) + leverX)*sin(trajRoll)*cos(trajPitch)*cos(trajYaw) - (lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*cos(borePitch)*cos(boreYaw) + leverY)*sin(trajPitch)*cos(trajYaw);
        a(1, 8) = (-sin(trajPitch)*sin(trajRoll)*sin(trajYaw) - cos(trajRoll)*cos(trajYaw))*(lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*sin(boreYaw)*cos(borePitch) + leverX) + (-sin(trajPitch)*sin(trajYaw)*cos(trajRoll) + sin(trajRoll)*cos(trajYaw))*(-lidarDist*sin(borePitch)*sin(scanAngleFB) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB) - lidarDist*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR) + leverZ) - (lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*cos(borePitch)*cos(boreYaw) + leverY)*sin(trajYaw)*cos(trajPitch);
        a(1, 9) = (sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*(-lidarDist*(-sin(borePitch)*sin(boreRoll)*sin(boreYaw) - cos(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB)) + (sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*(lidarDist*sin(boreRoll)*cos(borePitch)*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleLR)*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)) + (-lidarDist*(-sin(borePitch)*sin(boreRoll)*cos(boreYaw) + sin(boreYaw)*cos(boreRoll))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB))*cos(trajPitch)*cos(trajYaw);
        a(1, 10) = (sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*(-lidarDist*sin(borePitch)*sin(scanAngleFB)*sin(boreYaw) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*sin(boreYaw)*cos(borePitch)*cos(scanAngleFB) - lidarDist*sin(boreYaw)*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR)) + (sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*(-lidarDist*sin(borePitch)*sin(boreRoll)*sin(scanAngleLR)*cos(scanAngleFB) + lidarDist*sin(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*sin(scanAngleFB)*cos(borePitch)) + (-lidarDist*sin(borePitch)*sin(scanAngleFB)*cos(boreYaw) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB)*cos(boreYaw) - lidarDist*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR)*cos(boreYaw))*cos(trajPitch)*cos(trajYaw);
        a(1, 11) = (sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*(lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*cos(borePitch)*cos(boreYaw)) + (lidarDist*(-sin(borePitch)*sin(boreRoll)*sin(boreYaw) - cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(-sin(borePitch)*sin(boreYaw)*cos(boreRoll) + sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*sin(scanAngleFB)*sin(boreYaw)*cos(borePitch))*cos(trajPitch)*cos(trajYaw);
        a(1, 12) = sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll);
        a(1, 13) = cos(trajPitch)*cos(trajYaw);
        a(1, 14) = sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw);
        // partials with respect to lidar point Z
        a(2, 0) = (-sin(borePitch)*sin(scanAngleFB) + sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB) - cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR))*cos(trajPitch)*cos(trajRoll) + ((sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) + (-sin(borePitch)*sin(boreYaw)*cos(boreRoll) + sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + sin(scanAngleFB)*sin(boreYaw)*cos(borePitch))*sin(trajRoll)*cos(trajPitch) + (-(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleLR)*cos(scanAngleFB) - (-sin(borePitch)*cos(boreRoll)*cos(boreYaw) - sin(boreRoll)*sin(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) - sin(scanAngleFB)*cos(borePitch)*cos(boreYaw))*sin(trajPitch);
        a(2, 1) = (lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB))*sin(trajRoll)*cos(trajPitch) + (-lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB))*sin(trajPitch) + (lidarDist*sin(boreRoll)*cos(borePitch)*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleLR)*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB))*cos(trajPitch)*cos(trajRoll);
        a(2, 2) = (-lidarDist*sin(borePitch)*cos(scanAngleFB) - lidarDist*sin(boreRoll)*sin(scanAngleFB)*sin(scanAngleLR)*cos(borePitch) + lidarDist*sin(scanAngleFB)*cos(borePitch)*cos(boreRoll)*cos(scanAngleLR))*cos(trajPitch)*cos(trajRoll) + (-lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleFB)*sin(scanAngleLR) + lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*sin(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(boreYaw)*cos(borePitch)*cos(scanAngleFB))*sin(trajRoll)*cos(trajPitch) + (lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleFB)*sin(scanAngleLR) - lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*sin(scanAngleFB)*cos(scanAngleLR) - lidarDist*cos(borePitch)*cos(scanAngleFB)*cos(boreYaw))*sin(trajPitch);
        a(2, 3) = 0;
        a(2, 4) = 0;
        a(2, 5) = 1;
        a(2, 6) = -(-lidarDist*sin(borePitch)*sin(scanAngleFB) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB) - lidarDist*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR) + leverZ)*sin(trajRoll)*cos(trajPitch) + (lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*sin(boreYaw)*cos(borePitch) + leverX)*cos(trajPitch)*cos(trajRoll);
        a(2, 7) = -(-lidarDist*sin(borePitch)*sin(scanAngleFB) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB) - lidarDist*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR) + leverZ)*sin(trajPitch)*cos(trajRoll) - (lidarDist*(sin(borePitch)*sin(boreRoll)*sin(boreYaw) + cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*sin(boreYaw)*cos(borePitch) + leverX)*sin(trajPitch)*sin(trajRoll) + (-lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleLR)*cos(scanAngleFB) + lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*sin(scanAngleFB)*cos(borePitch)*cos(boreYaw) - leverY)*cos(trajPitch);
        a(2, 8) = 0;
        a(2, 9) = (-lidarDist*(-sin(borePitch)*sin(boreRoll)*sin(boreYaw) - cos(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(borePitch)*sin(boreYaw)*cos(boreRoll) - sin(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB))*sin(trajRoll)*cos(trajPitch) + (lidarDist*(-sin(borePitch)*sin(boreRoll)*cos(boreYaw) + sin(boreYaw)*cos(boreRoll))*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB))*sin(trajPitch) + (lidarDist*sin(boreRoll)*cos(borePitch)*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleLR)*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB))*cos(trajPitch)*cos(trajRoll);
        a(2, 10) = (-lidarDist*sin(borePitch)*sin(scanAngleFB)*sin(boreYaw) + lidarDist*sin(boreRoll)*sin(scanAngleLR)*sin(boreYaw)*cos(borePitch)*cos(scanAngleFB) - lidarDist*sin(boreYaw)*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR))*sin(trajRoll)*cos(trajPitch) + (lidarDist*sin(borePitch)*sin(scanAngleFB)*cos(boreYaw) - lidarDist*sin(boreRoll)*sin(scanAngleLR)*cos(borePitch)*cos(scanAngleFB)*cos(boreYaw) + lidarDist*cos(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR)*cos(boreYaw))*sin(trajPitch) + (-lidarDist*sin(borePitch)*sin(boreRoll)*sin(scanAngleLR)*cos(scanAngleFB) + lidarDist*sin(borePitch)*cos(boreRoll)*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*sin(scanAngleFB)*cos(borePitch))*cos(trajPitch)*cos(trajRoll);
        a(2, 11) = (-lidarDist*(-sin(borePitch)*sin(boreRoll)*sin(boreYaw) - cos(boreRoll)*cos(boreYaw))*sin(scanAngleLR)*cos(scanAngleFB) + lidarDist*(-sin(borePitch)*sin(boreYaw)*cos(boreRoll) + sin(boreRoll)*cos(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*sin(boreYaw)*cos(borePitch))*sin(trajPitch) + (lidarDist*(sin(borePitch)*sin(boreRoll)*cos(boreYaw) - sin(boreYaw)*cos(boreRoll))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(borePitch)*cos(boreRoll)*cos(boreYaw) + sin(boreRoll)*sin(boreYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*cos(borePitch)*cos(boreYaw))*sin(trajRoll)*cos(trajPitch);
        a(2, 12) = sin(trajRoll)*cos(trajPitch);
        a(2, 13) = -sin(trajPitch);
        a(2, 14) = cos(trajPitch)*cos(trajRoll);


        // // Jacobian matrix ordered the same as the observation covariance matrix
        // MatrixXd a = MatrixXd::Zero(3, 9);
        // // partials with respect to lidar point X
        // a(0, 0) = (sin(trajPitch)*sin(trajRoll)*sin(trajYaw) + cos(trajRoll)*cos(trajYaw))*sin(scanAngleLR)*cos(scanAngleFB) + (-sin(trajPitch)*sin(trajYaw)*cos(trajRoll) + sin(trajRoll)*cos(trajYaw))*cos(scanAngleFB)*cos(scanAngleLR) + sin(scanAngleFB)*sin(trajYaw)*cos(trajPitch);
        // a(0, 1) = lidarDist*(sin(trajPitch)*sin(trajRoll)*sin(trajYaw) + cos(trajRoll)*cos(trajYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(trajPitch)*sin(trajYaw)*cos(trajRoll) - sin(trajRoll)*cos(trajYaw))*sin(scanAngleLR)*cos(scanAngleFB);
        // a(0, 2) = -lidarDist*(sin(trajPitch)*sin(trajRoll)*sin(trajYaw) + cos(trajRoll)*cos(trajYaw))*sin(scanAngleFB)*sin(scanAngleLR) + lidarDist*(sin(trajPitch)*sin(trajYaw)*cos(trajRoll) - sin(trajRoll)*cos(trajYaw))*sin(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(trajYaw)*cos(trajPitch)*cos(scanAngleFB);
        // a(0, 3) = 1;
        // a(0, 4) = 0;
        // a(0, 5) = 0;
        // a(0, 6) = -lidarDist*(-sin(trajPitch)*sin(trajRoll)*sin(trajYaw) - cos(trajRoll)*cos(trajYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(trajPitch)*sin(trajYaw)*cos(trajRoll) - sin(trajRoll)*cos(trajYaw))*sin(scanAngleLR)*cos(scanAngleFB);
        // a(0, 7) = -lidarDist*sin(trajPitch)*sin(scanAngleFB)*sin(trajYaw) + lidarDist*sin(trajRoll)*sin(scanAngleLR)*sin(trajYaw)*cos(trajPitch)*cos(scanAngleFB) - lidarDist*sin(trajYaw)*cos(trajPitch)*cos(trajRoll)*cos(scanAngleFB)*cos(scanAngleLR);
        // a(0, 8) = lidarDist*(sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleFB)*cos(trajPitch)*cos(trajYaw);
        // // partials with respect to lidar point Y
        // a(1, 0) = (sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*sin(scanAngleLR)*cos(scanAngleFB) + (-sin(trajPitch)*cos(trajRoll)*cos(trajYaw) - sin(trajRoll)*sin(trajYaw))*cos(scanAngleFB)*cos(scanAngleLR) + sin(scanAngleFB)*cos(trajPitch)*cos(trajYaw);
        // a(1, 1) = lidarDist*(sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*sin(scanAngleLR)*cos(scanAngleFB);
        // a(1, 2) = -lidarDist*(sin(trajPitch)*sin(trajRoll)*cos(trajYaw) - sin(trajYaw)*cos(trajRoll))*sin(scanAngleFB)*sin(scanAngleLR) + lidarDist*(sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*sin(scanAngleFB)*cos(scanAngleLR) + lidarDist*cos(trajPitch)*cos(scanAngleFB)*cos(trajYaw);
        // a(1, 3) = 0;
        // a(1, 4) = 1;
        // a(1, 5) = 0;
        // a(1, 6) = -lidarDist*(-sin(trajPitch)*sin(trajRoll)*cos(trajYaw) + sin(trajYaw)*cos(trajRoll))*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*(sin(trajPitch)*cos(trajRoll)*cos(trajYaw) + sin(trajRoll)*sin(trajYaw))*sin(scanAngleLR)*cos(scanAngleFB);
        // a(1, 7) = -lidarDist*sin(trajPitch)*sin(scanAngleFB)*cos(trajYaw) + lidarDist*sin(trajRoll)*sin(scanAngleLR)*cos(trajPitch)*cos(scanAngleFB)*cos(trajYaw) - lidarDist*cos(trajPitch)*cos(trajRoll)*cos(scanAngleFB)*cos(scanAngleLR)*cos(trajYaw);
        // a(1, 8) = lidarDist*(-sin(trajPitch)*sin(trajRoll)*sin(trajYaw) - cos(trajRoll)*cos(trajYaw))*sin(scanAngleLR)*cos(scanAngleFB) - lidarDist*(-sin(trajPitch)*sin(trajYaw)*cos(trajRoll) + sin(trajRoll)*cos(trajYaw))*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*sin(scanAngleFB)*sin(trajYaw)*cos(trajPitch);
        // // partials with respect to lidar point Z
        // a(2, 0) = -sin(trajPitch)*sin(scanAngleFB) + sin(trajRoll)*sin(scanAngleLR)*cos(trajPitch)*cos(scanAngleFB) - cos(trajPitch)*cos(trajRoll)*cos(scanAngleFB)*cos(scanAngleLR);
        // a(2, 1) = lidarDist*sin(trajRoll)*cos(trajPitch)*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleLR)*cos(trajPitch)*cos(trajRoll)*cos(scanAngleFB);
        // a(2, 2) = -lidarDist*sin(trajPitch)*cos(scanAngleFB) - lidarDist*sin(trajRoll)*sin(scanAngleFB)*sin(scanAngleLR)*cos(trajPitch) + lidarDist*sin(scanAngleFB)*cos(trajPitch)*cos(trajRoll)*cos(scanAngleLR);
        // a(2, 3) = 0;
        // a(2, 4) = 0;
        // a(2, 5) = 1;
        // a(2, 6) = lidarDist*sin(trajRoll)*cos(trajPitch)*cos(scanAngleFB)*cos(scanAngleLR) + lidarDist*sin(scanAngleLR)*cos(trajPitch)*cos(trajRoll)*cos(scanAngleFB);
        // a(2, 7) = -lidarDist*sin(trajPitch)*sin(trajRoll)*sin(scanAngleLR)*cos(scanAngleFB) + lidarDist*sin(trajPitch)*cos(trajRoll)*cos(scanAngleFB)*cos(scanAngleLR) - lidarDist*sin(scanAngleFB)*cos(trajPitch);
        // a(2, 8) = 0;

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
            9. sensor yaw
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
               allows us to add the beam uncertainty in both left/right and 
               forward/back directions
        */

        MatrixXd c = MatrixXd::Zero(15, 15);

        // 1. lidar distance
        if (m_includeIncidenceAngle)
        {
            c(0, 0) = pow(m_sLidarDistance, 2.0)
                    + pow((lidarDist * tan(incidenceAngle) * m_laserBeamDivergence/4), 2.0);
        }
        else
        {
            c(0, 0) = pow(m_sLidarDistance, 2.0);
        }
        // 2. scan angle (right-left)
        c(1, 1) = pow(m_sScanAngle, 2.0) + pow(m_laserBeamDivergence/4, 2.0);
        // 3. scan angle (forward-back)
        c(2, 2) = pow(m_laserBeamDivergence/4, 2.0);
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
        // 10. boresight roll
        c(9, 9) = pow(m_sBoreRollPitch, 2.0);
        // 11. boresight pitch
        c(10, 10) = pow(m_sBoreRollPitch, 2.0);
        // 12. boresight yaw
        c(11, 11) = pow(m_sBoreYaw, 2.0);
        // 13. lever x
        c(12, 12) = pow(m_sLeverX, 2.0);
        // 14. lever y
        c(13, 13) = pow(m_sLeverY, 2.0);
        // 15. lever z
        c(14, 14) = pow(m_sLeverY, 2.0);

        return c;
    }


    void ALS_TPU::invertObservations(
        PointRef cloudPoint,
        double trajX, double trajY, double trajZ,
        double trajHeading, double trajPitch,
        double& lidarDist, double& scanAngleRL, double& scanAngleFB,
        double& incidenceAngle)
    {
        IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

        Vector3d laserVector;
        laserVector << (cloudPoint.getFieldAs<double>(Id::X) - trajX),
                       (cloudPoint.getFieldAs<double>(Id::Y) - trajY),
                       (cloudPoint.getFieldAs<double>(Id::Z) - trajZ);
        lidarDist = laserVector.norm();

        Vector3d groundNormalVector, unitLaserVector;
        groundNormalVector << cloudPoint.getFieldAs<double>(Id::NormalX),
                              cloudPoint.getFieldAs<double>(Id::NormalY),
                              cloudPoint.getFieldAs<double>(Id::NormalZ);
        unitLaserVector = laserVector / lidarDist;
        incidenceAngle = acos(unitLaserVector.dot(-groundNormalVector));
        if (incidenceAngle > m_maximumIncidenceAngle)
        {
            incidenceAngle = m_maximumIncidenceAngle;
        }

        // estimate scan angle (ignoring any forward/back angles for now)
        Vector3d unitTrajVector, unitPitchVector, unitNormalVector;
        // unit vector in direction of trajectory
        unitTrajVector << sin(trajHeading), cos(trajHeading), 0.0;
        // upward pointing unit pitch vector
        unitPitchVector << 0.0, 0.0, 1.0;
        // vector normal to plane created by trajectory and pitch vectors
        unitNormalVector = unitTrajVector.cross(unitPitchVector);
        // rotate unitPitchVector by pitch angle about the unitNormalVector
        // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
        unitPitchVector = cos(trajPitch) * unitPitchVector + sin(trajPitch) * unitNormalVector.cross(unitPitchVector);
        // estimate scan angle as angle between laser vector and pitch vector
        scanAngleRL = acos(unitLaserVector.dot(-unitPitchVector));
        // check which side of the trajectory vector the laser vector is pointing for sign
        scanAngleRL = copysign(scanAngleRL, unitLaserVector.dot(unitNormalVector));

        // for now
        scanAngleFB = 0.0;
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
