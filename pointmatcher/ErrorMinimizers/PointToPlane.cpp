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

#include <iostream>

#include "Eigen/SVD"

#include "ErrorMinimizersImpl.h"
#include "PointMatcherPrivate.h"
#include "Functions.h"

using namespace Eigen;
using namespace std;

typedef PointMatcherSupport::Parametrizable Parametrizable;
typedef PointMatcherSupport::Parametrizable P;
typedef Parametrizable::Parameters Parameters;
typedef Parametrizable::ParameterDoc ParameterDoc;
typedef Parametrizable::ParametersDoc ParametersDoc;

template<typename T>
PointToPlaneErrorMinimizer<T>::PointToPlaneErrorMinimizer(const Parameters& params):
	ErrorMinimizer(name(), availableParameters(), params),
	force2D(Parametrizable::get<T>("force2D")),
    force4DOF(Parametrizable::get<T>("force4DOF"))
{
	if(force2D)
		{
			if (force4DOF)
			{
				throw PointMatcherSupport::ConfigurationError("Force 2D cannot be used together with force4DOF.");
			}
			else
			{
				LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 2D.");
			}
		}
	else if(force4DOF)
	{
	 	LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 4-DOF (yaw,x,y,z).");
	}
	else
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 3D.");
	}
}

template<typename T>
PointToPlaneErrorMinimizer<T>::PointToPlaneErrorMinimizer(const ParametersDoc paramsDoc, const Parameters& params):
	ErrorMinimizer(name(), paramsDoc, params),
	force2D(Parametrizable::get<T>("force2D")),
	force4DOF(Parametrizable::get<T>("force4DOF"))
{
	if(force2D)
	{
		if (force4DOF)
		{
			throw PointMatcherSupport::ConfigurationError("Force 2D cannot be used together with force4DOF.");
		}
		else
		{
			LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 2D.");
		}
	}
	else if(force4DOF)
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 4-DOF (yaw,x,y,z).");
	}
	else
	{
		LOG_INFO_STREAM("PointMatcher::PointToPlaneErrorMinimizer - minimization will be in 3D.");
	}
}

template<typename T>
void PointToPlaneErrorMinimizer<T>::formulatePointMatchingConstraints(const ErrorElements& mPts, Matrix& A, Vector& b) const
{
    // This function follows the notation and steps from the reference material (see Doxygen).

    const Index dim{ mPts.reading.features.rows() };

    // Fetch normal vectors of the reference point cloud (with adjustment if needed)
    auto normalRef{ mPts.reference.getDescriptorViewByName("normals").topRows(dim - 1) };

    // Note: Normal vector must be precalculated to use this error. Use appropriate input filter.
    assert(normalRef.rows() > 0);

    // Compute cross product of cross = cross(reading X normalRef)
    Matrix cross;
    Matrix matrixGamma(3, 3);
    if (!force4DOF)
    {
        // Compute cross product of cross = cross(reading X normalRef)
        cross = this->crossProduct(mPts.reading.features, normalRef);
    }
    else
    {
        //VK: Instead for "cross" as in 3D, we need only a dot product with the matrixGamma factor for 4DOF
        //VK: This should be published in 2020 or 2021
        matrixGamma << 0, -1, 0, 1, 0, 0, 0, 0, 0;
        cross = ((matrixGamma * mPts.reading.features).transpose() * normalRef).diagonal().transpose();
    }

    /* Form G, the matrix of stacked rotational and translation constraints */
    // G  = [cross, normals]
    Matrix G(normalRef.rows() + cross.rows(), normalRef.cols());
    G << cross, normalRef;

    /* Form h, the matrix of residuals*/
    // h is the sum of of dot(deltas, normals)
    const Matrix deltas{ mPts.reading.features - mPts.reference.features };
    Matrix h{ Matrix::Zero(1, normalRef.cols()) };
    for (Index i{ 0 }; i < normalRef.rows(); ++i)
    {
        h += (deltas.row(i).array() * normalRef.row(i).array()).matrix();
    }

    /* Constraint formulation for Ax=b */
    // In the reference material the formulation is   (G * G') * Tau = G * h   , hence:
    // Unadjusted covariance A = G * G'
    A = G * G.transpose();
    // b = -(G' * dot)
    b = -(G * h.transpose());
}

template<typename T, typename MatrixA, typename Vector>
bool solveLinearSystem(const MatrixA& A, const Vector& b, Vector& x)
{
    static constexpr T kNumericalErrorTolerance{ 1e-5 };

    assert(A.cols() == A.rows());
    assert(b.cols() == 1);
    assert(b.rows() == A.rows());
    assert(x.cols() == 1);
    assert(x.rows() == A.cols());

    // SVD, exists for all matrices without exception. If there is a solution it will find it despite not being optimum.
    x = A.template cast<double>().jacobiSvd(ComputeThinU | ComputeThinV).solve(b.template cast<double>()).template cast<T>();

    const Vector ax{(A * x).eval()};
    if ((b - ax).norm() > kNumericalErrorTolerance * std::max(A.norm() * x.norm(), b.norm()))
    {
        LOG_WARNING_STREAM("PointMatcher::icp - encountered numerically singular matrix while minimizing point to plane "
                           "distance."
                           << " b=" << b.transpose() << " !~ A * x=" << (ax).transpose().eval() << ": ||b- ax||=" << (b - ax).norm()
                           << ", ||b||=" << b.norm() << ", ||ax||=" << ax.norm());
        return false;
    }

    return true;
}

template<typename T, typename MatrixA, typename Vector>
void solvePossiblyUnderdeterminedLinearSystem(const MatrixA& A, const Vector & b, Vector & x) {
	assert(A.cols() == A.rows());
	assert(b.cols() == 1);
	assert(b.rows() == A.rows());
	assert(x.cols() == 1);
	assert(x.rows() == A.cols());

	typedef typename PointMatcher<T>::Matrix Matrix;

	BOOST_AUTO(Aqr, A.fullPivHouseholderQr());
	if (!Aqr.isInvertible())
	{
		// Solve reduced problem R1 x = Q1^T b instead of QR x = b, where Q = [Q1 Q2] and R = [ R1 ; R2 ] such that ||R2|| is small (or zero) and therefore A = QR ~= Q1 * R1
		const int rank = Aqr.rank();
		const int rows = A.rows();
		const Matrix Q1t = Aqr.matrixQ().transpose().block(0, 0, rank, rows);
		const Matrix R1 = (Q1t * A * Aqr.colsPermutation()).block(0, 0, rank, rows);

		const bool findMinimalNormSolution = true; // TODO is that what we want?

		// The under-determined system R1 x = Q1^T b is made unique ..
		if(findMinimalNormSolution){
			// by getting the solution of smallest norm (x = R1^T * (R1 * R1^T)^-1 Q1^T b.
			x = R1.template triangularView<Eigen::Upper>().transpose() * (R1 * R1.transpose()).llt().solve(Q1t * b);
		} else {
			// by solving the simplest problem that yields fewest nonzero components in x
			x.block(0, 0, rank, 1) = R1.block(0, 0, rank, rank).template triangularView<Eigen::Upper>().solve(Q1t * b);
			x.block(rank, 0, rows - rank, 1).setZero();
		}

		x = Aqr.colsPermutation() * x;

		BOOST_AUTO(ax , (A * x).eval());
		if (!b.isApprox(ax, 1e-5)) {
			LOG_INFO_STREAM("PointMatcher::icp - encountered almost singular matrix while minimizing point to plane distance. QR solution was too inaccurate. Trying more accurate approach using double precision SVD.");
			x = A.template cast<double>().jacobiSvd(ComputeThinU | ComputeThinV).solve(b.template cast<double>()).template cast<T>();
			ax = A * x;

			if((b - ax).norm() > 1e-5 * std::max(A.norm() * x.norm(), b.norm())){
				LOG_WARNING_STREAM("PointMatcher::icp - encountered numerically singular matrix while minimizing point to plane distance and the current workaround remained inaccurate."
						<< " b=" << b.transpose()
						<< " !~ A * x=" << (ax).transpose().eval()
						<< ": ||b- ax||=" << (b - ax).norm()
						<< ", ||b||=" << b.norm()
						<< ", ||ax||=" << ax.norm());
			}
		}
	}
	else {
		// Cholesky decomposition
		x = A.llt().solve(b);
	}
}

template<typename T>
typename PointMatcher<T>::TransformationParameters PointToPlaneErrorMinimizer<T>::compute(ErrorElements& mPts)
{
    // TODO(ynava) Rename mPts to matchedPoints.
    const int dim = mPts.reading.features.rows();
    const int nbPts = mPts.reading.features.cols();

    // Adjust if the user forces 2D minimization on XY-plane
    if (force2D && dim == 4)
    {
        mPts.reading.features.conservativeResize(3, Eigen::NoChange);
        mPts.reading.features.row(2) = Matrix::Ones(1, nbPts);
        mPts.reference.features.conservativeResize(3, Eigen::NoChange);
        mPts.reference.features.row(2) = Matrix::Ones(1, nbPts);
    }

    // This ICP implementation is capable of estimating in
    // - 2D
    // - 3D
    //   * 6DOF (full pose)
    //   * 4DOF (pose without tilt)

    // Compute the mean of the matched reading and reference point clouds, which represents their Center Of Mass.
    const Vector meanReadingMatches{ mPts.reading.features.topRows(dim - 1).rowwise().mean() };
    const Vector meanReferenceMatches{ mPts.reference.features.topRows(dim - 1).rowwise().mean() };

    // Shift the reading and reference point clouds by their mean.
    mPts.reading.features.topRows(dim - 1).colwise() -= meanReadingMatches;
    mPts.reference.features.topRows(dim - 1).colwise() -= meanReferenceMatches;

    // Formulate optimization constraints based on point-matching.
    Matrix A;
    Vector b;
    formulatePointMatchingConstraints(mPts, A, b);

    // Solution vector x, consisting of linearized versions of rotation angles (for 2D [gamma] and for 3D [pitch, roll, yaw])
    // stacked on top of translational coordinates (for 2D [x,y] and for 3D [x,y,z])
    Vector x(A.rows());

    // Solve the problem. This is the gist of the optimization where we run least squares to minimize the cost function.
    solvePossiblyUnderdeterminedLinearSystem<T>(A, b, x);

    // Transform parameters to matrix
    Matrix mOut;
    if (dim == 4 && !force2D)
    {
        /* 3D ICP */

        // Solution vector x = [alpha, beta, gamma, x, y, z]
        Eigen::Transform<T, 3, Eigen::Affine> T_refMeanMatches_readMeanMatches{ Eigen::Transform<T, 3, Eigen::Affine>::Identity() };

        // Normal 6DOF takes the whole rotation vector from the solution to construct the output quaternion
        if (!force4DOF)
        {
            const T rotationAngle{ x.head(3).norm() }; // Solution vector x = [<alpha, beta, gamma>, x, y, z]
            T_refMeanMatches_readMeanMatches = Eigen::AngleAxis<T>(rotationAngle, x.head(3).stableNormalized());
        }
        else // 4DOF needs only one number, the rotation around the Z axis
        {
            Vector unitZ(3, 1);
            unitZ << 0, 0, 1;
            const T rotationAngle{ x(0) }; // Solution vector x = [<gamma>, x, y, z]
            T_refMeanMatches_readMeanMatches = Eigen::AngleAxis<T>(rotationAngle, unitZ); //x=[<gamma>,x,y,z]
        }

        // Translation.
        if (!force4DOF)
        {
            T_refMeanMatches_readMeanMatches.translation() = x.segment(3, 3); //x=[alpha,beta,gamma,<x,y,z>]
        }
        else
        {
            T_refMeanMatches_readMeanMatches.translation() = x.segment(1, 3); //x=[gamma,<x,y,z>]
        }

        // Compute the transformations from reading/reference input frame to their mean.
        Eigen::Transform<T, 3, Eigen::Affine> T_refIn_refMeanMatches{ Eigen::Transform<T, 3, Eigen::Affine>::Identity() };
        T_refIn_refMeanMatches.translation() = meanReferenceMatches;
        Eigen::Transform<T, 3, Eigen::Affine> T_readIn_readMeanMatches{ Eigen::Transform<T, 3, Eigen::Affine>::Identity() };
        T_readIn_readMeanMatches.translation() = meanReadingMatches;

        // Move ICP transformation back into the input reference frame, removing the mean / CoM offset.
        const Eigen::Transform<T, 3, Eigen::Affine> T_refIn_readIn{ T_refIn_refMeanMatches * T_refMeanMatches_readMeanMatches
                                                                    * T_readIn_readMeanMatches.inverse() };
        mOut = T_refIn_readIn.matrix();

        if (mOut != mOut)
        {
            // Degenerate situation. This can happen when the source and reading clouds
            // are identical, and then b and x above are 0, and the rotation matrix cannot
            // be determined, it comes out full of NaNs. The correct rotation is the identity.
            mOut.block(0, 0, dim - 1, dim - 1) = Matrix::Identity(dim - 1, dim - 1);
        }
    }
    else
    {
        /* 2D ICP */

        // Solution vector x = [gamma, x, y]
        Eigen::Transform<T, 2, Eigen::Affine> T_refMeanMatches_readMeanMatches{ Eigen::Transform<T, 2, Eigen::Affine>::Identity() };

        const T rotationAngle{ x(0) }; // Solution vector x = [<gamma>, x, y, z]
        T_refMeanMatches_readMeanMatches = Eigen::Rotation2D<T>(rotationAngle);

        T_refMeanMatches_readMeanMatches.translation() = x.segment(1, 2); //x=[gamma,<x,y>]

        // Compute the transformations from reading/reference input frame to their mean.
        Eigen::Transform<T, 2, Eigen::Affine> T_refIn_refMeanMatches{ Eigen::Transform<T, 2, Eigen::Affine>::Identity() };
        T_refIn_refMeanMatches.translation() = meanReferenceMatches;
        Eigen::Transform<T, 2, Eigen::Affine> T_readIn_readMeanMatches{ Eigen::Transform<T, 2, Eigen::Affine>::Identity() };
        T_readIn_readMeanMatches.translation() = meanReadingMatches;

        // Move ICP transformation back into the input reference frame, removing the mean / CoM offset.
        const Eigen::Transform<T, 2, Eigen::Affine> T_refIn_readIn{ T_refIn_refMeanMatches * T_refMeanMatches_readMeanMatches
                                                                    * T_readIn_readMeanMatches.inverse() };

        if (force2D)
        {
            mOut = Matrix::Identity(dim, dim);
            mOut.topLeftCorner(2, 2) = T_refIn_readIn.matrix().topLeftCorner(2, 2);
            mOut.topRightCorner(2, 1) = T_refIn_readIn.matrix().topRightCorner(2, 1);
        }
        else
        {
            mOut = T_refIn_readIn.matrix();
        }
    }
    return mOut;
}


template<typename T>
T PointToPlaneErrorMinimizer<T>::computeResidualError(ErrorElements mPts, const bool& force2D)
{
	const int dim = mPts.reading.features.rows();
	const int nbPts = mPts.reading.features.cols();

	// Adjust if the user forces 2D minimization on XY-plane
	int forcedDim = dim - 1;
	if(force2D && dim == 4)
	{
		mPts.reading.features.conservativeResize(3, Eigen::NoChange);
		mPts.reading.features.row(2) = Matrix::Ones(1, nbPts);
		mPts.reference.features.conservativeResize(3, Eigen::NoChange);
		mPts.reference.features.row(2) = Matrix::Ones(1, nbPts);
		forcedDim = dim - 2;
	}

	// Fetch normal vectors of the reference point cloud (with adjustment if needed)
	const BOOST_AUTO(normalRef, mPts.reference.getDescriptorViewByName("normals").topRows(forcedDim));

	// Note: Normal vector must be precalculated to use this error. Use appropriate input filter.
	assert(normalRef.rows() > 0);

	const Matrix deltas = mPts.reading.features - mPts.reference.features;

	// dotProd = dot(deltas, normals) = d.n
	Matrix dotProd = Matrix::Zero(1, normalRef.cols());
	for(int i = 0; i < normalRef.rows(); i++)
	{
		dotProd += (deltas.row(i).array() * normalRef.row(i).array()).matrix();
	}
	// residual = w*(d.n)²
	dotProd = (mPts.weights.row(0).array() * dotProd.array().square()).matrix();

	// return sum of the norm of each dot product
	return dotProd.sum();
}


template<typename T>
T PointToPlaneErrorMinimizer<T>::getResidualError(
	const DataPoints& filteredReading,
	const DataPoints& filteredReference,
	const OutlierWeights& outlierWeights,
	const Matches& matches) const
{
	assert(matches.ids.rows() > 0);

	// Fetch paired points
	typename ErrorMinimizer::ErrorElements mPts(filteredReading, filteredReference, outlierWeights, matches);

	return PointToPlaneErrorMinimizer::computeResidualError(mPts, force2D);
}

template<typename T>
T PointToPlaneErrorMinimizer<T>::getOverlap() const
{

	// Gather some information on what kind of point cloud we have
	const bool hasReferenceDensity = this->lastErrorElements.reference.descriptorExists("densities");

	const int nbPoints = this->lastErrorElements.reading.features.cols();
	const int dim = this->lastErrorElements.reading.features.rows();

	// basix safety check
	if(nbPoints == 0)
	{
		throw std::runtime_error("Error, last error element empty. Error minimizer needs to be called at least once before using this method.");
	}

	// optimal case
	if (hasReferenceDensity)
	{
		// find median density

		Matrix densities = this->lastErrorElements.reference.getDescriptorViewByName("densities");
		vector<T> values(densities.data(), densities.data() + densities.size());

		// sort up to half the values
		nth_element(values.begin(), values.begin() + (values.size() * 0.5), values.end());

		// extract median value
		const T medianDensity = values[values.size() * 0.5];
		const T medianRadius = 1.0/pow(medianDensity, 1/3.0);
	}
	else
	{
		LOG_INFO_STREAM("PointToPlaneErrorMinimizer - warning, no sensor noise and density. Using best estimate given outlier rejection instead.");
		return this->getWeightedPointUsedRatio();
	}


	const Vector dists = (this->lastErrorElements.reading.features.topRows(dim-1) - this->lastErrorElements.reference.features.topRows(dim-1)).colwise().norm();


	// here we can only loop through a list of links, but we are interested in whether or not
	// a point has at least one valid match.
	int count = 0;
	int nbUniquePoint = 1;
	Vector lastValidPoint = this->lastErrorElements.reading.features.col(0) * 2.;
	for(int i=0; i < nbPoints; i++)
	{
		const Vector point = this->lastErrorElements.reading.features.col(i);

		if(lastValidPoint != point)
		{
			// NOTE: we tried with the projected distance over the normal vector before:
			// projectionDist = delta dotProduct n.normalized()
			// but this doesn't make sense 

			lastValidPoint = point;
			count++;
		}

		// Count unique points
		if(i > 0)
		{
			if(point != this->lastErrorElements.reading.features.col(i-1))
				nbUniquePoint++;
		}

	}
	//cout << "count: " << count << ", nbUniquePoint: " << nbUniquePoint << ", this->lastErrorElements.nbRejectedPoints: " << this->lastErrorElements.nbRejectedPoints << endl;

	return (T)count/(T)(nbUniquePoint + this->lastErrorElements.nbRejectedPoints);
}

template struct PointToPlaneErrorMinimizer<float>;
template struct PointToPlaneErrorMinimizer<double>;
