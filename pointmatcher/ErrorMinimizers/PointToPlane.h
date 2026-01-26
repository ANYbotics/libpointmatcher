// kate: replace-tabs off; indent-width 4; indent-mode normal
// vim: ts=4:sw=4:noexpandtab
/*

Copyright (c) 2010--2012,
François Pomerleau and Stephane Magnenat, ASL, ETHZ, Switzerland
You can contact the authors at <f dot pomerleau at gmail dot com> and
<stephane at magnenat dot net>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETH-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef POINT_TO_PLANE_ERROR_MINIMIZER_H
#define POINT_TO_PLANE_ERROR_MINIMIZER_H

#include "PointMatcher.h"

template<typename T>
struct PointToPlaneErrorMinimizer: public PointMatcher<T>::ErrorMinimizer
{
    typedef PointMatcherSupport::Parametrizable Parametrizable;
    typedef PointMatcherSupport::Parametrizable P;
    typedef Parametrizable::Parameters Parameters;
    typedef Parametrizable::ParameterDoc ParameterDoc;
    typedef Parametrizable::ParametersDoc ParametersDoc;

    typedef typename PointMatcher<T>::ScalarType ScalarType;
    typedef typename PointMatcher<T>::DataPoints DataPoints;
    typedef typename PointMatcher<T>::DataPoints::Index Index;
    typedef typename PointMatcher<T>::Matches Matches;
    typedef typename PointMatcher<T>::OutlierWeights OutlierWeights;
    typedef typename PointMatcher<T>::ErrorMinimizer ErrorMinimizer;
    typedef typename PointMatcher<T>::ErrorMinimizer::ErrorElements ErrorElements;
    typedef typename PointMatcher<T>::TransformationParameters TransformationParameters;
    typedef typename PointMatcher<T>::Vector Vector;
    typedef typename PointMatcher<T>::Matrix Matrix;

	virtual inline const std::string name()
	{
		return "PointToPlaneErrorMinimizer";
	}

    inline static const std::string description()
    {
        return "Point-to-plane error. Per \\cite{Chen1991Point2Plane}.";
    }

    inline static const ParametersDoc availableParameters()
    {
        return {
                {"force4DOF", "If set to true(1), the minimization will optimize only yaw and translation, pitch and roll will follow the prior.", "0", "0", "1", &P::Comp<bool>}

        };
    }

    const bool force4DOF;

    PointToPlaneErrorMinimizer(const Parameters& params = Parameters());
    PointToPlaneErrorMinimizer(const ParametersDoc paramsDoc, const Parameters& params);

    //! Builds linear optimization constraints based on point-matching with a point-to-plane cost.
    //!  Reference: "A Review of Point Cloud Registration Algorithms for Mobile Robotics" by Pomerleau et al (2015),
    //! @param mPts[in]   Error Elements containing correspondences between reading and references.
    //! @param A[out]     Covariance matrix of the point-to-plane error. Size (6,6) for 3D.
    //! @param b[out]     Vector of weighted residuals. Size (6,1) for 3D.
    void formulatePointMatchingConstraints(const ErrorElements& mPts, Matrix& A, Vector& b) const;

    virtual TransformationParameters compute(ErrorElements& mPts);
    virtual T getResidualError(const DataPoints& filteredReading, const DataPoints& filteredReference, const OutlierWeights& outlierWeights, const Matches& matches) const;
    virtual T getOverlap() const;

    static T computeResidualError(ErrorElements mPts);
};

//! Solves a linear system of the form Ax=b employing the Jacobi SVD algorithm (https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html)
//!   This method makes no assumption on the properties of the constraint matrices. SVD exists for all matrices without exception. If there is
//!   a solution it will find it despite not being optimal.
//! @param A[in]  Covariance matrix of the errors.
//! @param b[in]  Matrix of weighted residuals.
//! @param x[out] Solution vector.
//! @return bool  If the linear system could be solved in a numerically stable manner, false otherwise.
template<typename T, typename MatrixA, typename Vector>
bool solveLinearSystem(const MatrixA& A, const Vector& b, Vector& x);

// TODO(ynava) Deprecate, behavior is not well-justified, as solving the linear problem is not a real performance bottleneck for point-to-plane ICP.
//! Solves a linear system of the form Ax=b, trying out different solvers, with the aim of using the fastest solver for the task.
template<typename T, typename MatrixA, typename Vector>
void solvePossiblyUnderdeterminedLinearSystem(const MatrixA& A, const Vector & b, Vector & x);

#endif
