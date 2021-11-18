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

#pragma once

#include <pdal/Filter.hpp>
#include <Eigen/Dense>

namespace pdal {

    class PDAL_DLL ALS_TPU : public Filter {
        public:
            ALS_TPU() : Filter(), m_cloud(nullptr), m_complete(false)
            {}
            std::string getName() const;

        private:
            std::string m_profileFile, m_stDevFile;

            double m_maximumIncidenceAngle;
            double m_noDataValue;
            bool m_includeIncidenceAngle;
            bool m_extendedOutput;

            double m_stdLidarRange = 0.0, m_stdScanAngle = 0.0;
            double m_stdSensorXy = 0.0, m_stdSensorZ = 0.0;
            double m_stdSensorRollPitch = 0.0, m_stdSensorYaw = 0.0;
            double m_stdBoreRollPitch = 0.0, m_stdBoreYaw = 0.0;
            double m_stdLeverXyz = 0.0;
            double m_beamDivergence = 0.0;

            Dimension::Id m_lidarRange, m_scanAngleRL, m_scanAngleFB, m_incAngle;
            Dimension::Id m_xVar, m_yVar, m_zVar, m_xyCov, m_xzCov, m_yzCov;
            Dimension::Id m_xStd, m_yStd, m_zStd;
            Dimension::Id m_trajX, m_trajY, m_trajZ, m_trajRoll, m_trajPitch, m_trajHeading;

            virtual void addArgs(ProgramArgs& args);
            void initialize();
            virtual void addDimensions(PointLayoutPtr layout);
            virtual PointViewSet run(PointViewPtr view);
            virtual void done(PointTableRef _);

            void setProfile();
            PointViewPtr tpu(PointViewPtr cloud, PointViewPtr trajectory);
            bool linearInterpolation(
                double pointTime,
                double& trajX, double& trajY, double& trajZ,
                double& trajHeading, double& trajPitch,
                PointViewPtr trajectory, PointId& interpIdx);
            void invertObservations(
                PointRef cloudPoint,
                double trajX, double trajY, double trajZ,
                double trajHeading, double trajPitch,
                double& lidarDist, double& scanAngleRL, double& scanAngleFB,
                double& incidenceAngle);
            Eigen::MatrixXd observationCovariance(
                double lidarDist, double incidenceAngle);
            Eigen::Matrix3d propagateCovariance(
                double lidarDist, double scanAngleLR, double scanAngleFB,
                double trajX, double trajY, double trajZ,
                double trajRoll, double trajPitch, double trajHeading,
                double boreRoll, double borePitch, double boreYaw,
                double leverX, double leverY, double leverZ,
                Eigen::MatrixXd obsCovariance);

            PointViewPtr m_cloud;
            bool m_complete;

            ALS_TPU& operator=(const ALS_TPU&);     // not implemented
            ALS_TPU(const ALS_TPU&);                // not implemented
    };

} // namespace pdal
