
#include "utils_transformations.h"

#include <random>

#include <Eigen/Core>

TransformationError computeError(const PM::AffineTransform& transformA, const PM::AffineTransform& transformB)
{
    TransformationError error;

    // Compute delta between A and B.
    const PM::AffineTransform deltaTransform{ transformA.inverse() * transformB };

    // Translation.
    error.deltaTranslation = deltaTransform.translation();
    error.translationNorm = error.deltaTranslation.norm();

    // Rotation.
    const PM::Quaternion deltaRotation{ deltaTransform.linear() };
    error.deltaRotationAngleAxis = Eigen::AngleAxis<PM::ScalarType>(deltaRotation);
    error.rotationAngle = std::abs(error.deltaRotationAngleAxis.angle());

    return error;
}

bool isApprox(const PM::AffineTransform& transformA, const PM::AffineTransform& transformB, const PM::ScalarType epsilon,
              std::string& message)
{
    const TransformationError error{ computeError(transformA, transformB) };

    // Translation
    std::stringstream messageStream;
    bool areApproxEqual{ true };
    if ((error.deltaTranslation.array().abs() >= epsilon).any())
    {
        messageStream << "Translation delta = " << error.deltaTranslation.transpose() << std::endl;
        messageStream << "Magnitude of translation delta = " << error.translationNorm << std::endl;
        areApproxEqual = false;
    }

    if (std::abs(error.rotationAngle) >= epsilon)
    {
        messageStream << "Angle-Axis angle = " << error.rotationAngle << std::endl;
        messageStream << "Angle-Axis axis = " << error.deltaRotationAngleAxis.axis().transpose() << std::endl;
        areApproxEqual = false;
    }

    messageStream << "Epsilon = " << epsilon << std::endl;
    message = messageStream.str();

    return areApproxEqual;
}