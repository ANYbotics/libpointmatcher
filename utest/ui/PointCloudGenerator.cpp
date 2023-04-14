
#include "../utest.h"
#include "utils_filesystem.h"

class PointCloudGeneratorTest : public ::testing::Test
{
public:
    PointCloudGeneratorTest() = default;

    /* Setup methods */
    void setDefaultParameters()
    {
        translation_ = PM::StaticCoordVector(0.0f, 0.5f, 0.0f);
        orientation_ = PM::Quaternion(0.0f, 0.2f, 5.0f, 1.0f);
        orientation_.normalize();

        numberOfPoints_ = 10000;
    }

    bool checkNormalsPointOutwards(const PM::DataPoints& pointCloud, const PM::Vector& center)
    {
        bool allNormalsPointOut{ true };
        auto normalsView{ pointCloud.getDescriptorViewByName("normals") };
        for (PM::DataPoints::Index i{ 0 }; i < pointCloud.features.cols(); ++i)
        {
            // Fetch point and remove transformation offset.
            const PM::Vector pointToCenterVector(pointCloud.features.col(i).head(3) - center);
            const PM::Vector normal{ normalsView.col(i) };
            const PM::ScalarType dotProduct{pointToCenterVector.dot(normal)};

            // We check if the normals of the shape are pointing inwards or outwards.
            if (dotProduct < 0)
            {
                allNormalsPointOut = false;
                break;
            }
        }

        return allNormalsPointOut;
    }

    bool dumpTestArtifacts(const std::string& testCase, const PM::DataPoints& pointCloud)
    {
        const std::string targetDirectory(testDataFolderPath + testCase + "/");

        // Remove directory, just in case it already exists.
        if (checkDirectoryExists(targetDirectory))
        {
            return removeDirectory(targetDirectory);
        }

        // Create directory.
        if (!createDirectory(targetDirectory))
        {
            return false;
        }

        pointCloud.save(targetDirectory + "point_cloud.ply");

        return true;
    }

    // Parameters.
    PM::StaticCoordVector translation_{ PM::StaticCoordVector::Zero() };
    PM::Quaternion orientation_{ PM::Quaternion::Identity() };
    PM::DataPoints::Index numberOfPoints_{ 0 };

    // Error tolerance.
    const PM::ScalarType kEpsilonError_{ 1e-5 };

    //! Whether test data should be saved to disk.
#ifdef SAVE_TEST_DATA_TO_DISK
    const bool saveTestDataToDisk{ SAVE_TEST_DATA_TO_DISK };
#else
    const bool saveTestDataToDisk{ false };
#endif
    const std::string testDataFolderPath{ "/tmp/libpointmatcher/unit_tests/point_cloud_generation/" };
};

// This test validates that the function that builds up transformations to point clouds is correct. Considers pure translation
TEST_F(PointCloudGeneratorTest, BuildUpTransformationTranslationOnly)
{ // NOLINT
    const PM::StaticCoordVector translation{ 1.0f, 0.5f, -50.212312f };
    const PM::Quaternion orientation{ 0.0f, 0.0f, 0.0f, 1.0f };

    // Build up transformation.
    const PM::AffineTransform transformation{ PM::PointCloudGenerator::buildUpTransformation(translation, orientation) };

    // Assertions on results.
    ASSERT_EQ(transformation.translation(), translation);
    ASSERT_EQ(transformation.linear(), orientation.normalized().toRotationMatrix());
}

// This test validates that the function that builds up transformations to point clouds is correct. Considers pure rotation.
TEST_F(PointCloudGeneratorTest, BuildUpTransformationRotationOnly)
{ // NOLINT
    const PM::StaticCoordVector translation{ 0.0f, 0.0f, 0.0f };
    const PM::Quaternion orientation{ 0.123123f, 0.9576f, -42.232193f, 0.00001f };

    // Build up transformation.
    const PM::AffineTransform transformation{ PM::PointCloudGenerator::buildUpTransformation(translation, orientation) };

    // Assertions on results.
    ASSERT_EQ(transformation.translation(), translation);
    ASSERT_EQ(transformation.linear(), orientation.normalized().toRotationMatrix());
}

// This test validates that the function that builds up transformations to point clouds is correct. Considers translation+rotation.
TEST_F(PointCloudGeneratorTest, BuildUpTransformationTranslationRotation)
{ // NOLINT
    const PM::StaticCoordVector translation{ 1.0f, 0.5f, -50.212312f };
    const PM::Quaternion orientation{ 0.123123f, 0.9576f, -42.232193f, 0.00001f };

    // Build up transformation.
    const PM::AffineTransform transformation{ PM::PointCloudGenerator::buildUpTransformation(translation, orientation) };

    // Assertions on results.
    ASSERT_EQ(transformation.translation(), translation);
    ASSERT_EQ(transformation.linear(), orientation.normalized().toRotationMatrix());
}

// This test validates that the function that creates empty 3D point clouds is correct.
TEST_F(PointCloudGeneratorTest, AddEmpty3dPointFields)
{ // NOLINT
    PM::DataPoints pointCloud;
    PM::PointCloudGenerator::addEmpty3dPointFields(numberOfPoints_, pointCloud);

    // Assertions on results.
    // Number of points.
    ASSERT_EQ(pointCloud.getNbPoints(), numberOfPoints_);
    // Feature labels.
    ASSERT_TRUE(pointCloud.featureExists(std::string("x")));
    ASSERT_TRUE(pointCloud.featureExists(std::string("y")));
    ASSERT_TRUE(pointCloud.featureExists(std::string("z")));
    ASSERT_TRUE(pointCloud.featureLabels.size());
}

// This test validates that the function that applies transformations to point clouds is correct.
TEST_F(PointCloudGeneratorTest, ApplyTransformation)
{ // NOLINT
    // Test points.
    const PM::DataPoints::Index numberOfPoints{ 2 };
    const PM::StaticCoordVector point1{ 0.0f, 0.0f, 0.0f };
    const PM::StaticCoordVector point2{ 2.1213f, -100000.0f, -23459999.2342312370987978687f };
    // First point is at the origin, second is somewhere else.

    // Point cloud.
    PM::DataPoints pointCloud;
    PM::PointCloudGenerator::addEmpty3dPointFields(numberOfPoints, pointCloud);
    pointCloud.features(0, 0) = point1(0);
    pointCloud.features(1, 0) = point1(1);
    pointCloud.features(2, 0) = point1(2);
    pointCloud.features(0, 1) = point2(0);
    pointCloud.features(1, 1) = point2(1);
    pointCloud.features(2, 1) = point2(2);

    // Build up transformation.
    const PM::AffineTransform transformation{ PM::PointCloudGenerator::buildUpTransformation(translation_, orientation_) };

    // Transform point cloud and points.
    PM::PointCloudGenerator::applyTransformation(translation_, orientation_, pointCloud);
    const PM::StaticCoordVector transformedPoint1{ transformation * point1 };
    const PM::StaticCoordVector transformedPoint2{ transformation * point2 };

    // Assertions on results.
    // Number of points.
    ASSERT_EQ(pointCloud.getNbPoints(), numberOfPoints);
    // Value of transformed points.
    ASSERT_EQ(pointCloud.features(0, 0), transformedPoint1(0));
    ASSERT_EQ(pointCloud.features(1, 0), transformedPoint1(1));
    ASSERT_EQ(pointCloud.features(2, 0), transformedPoint1(2));
    ASSERT_EQ(pointCloud.features(0, 1), transformedPoint2(0));
    ASSERT_EQ(pointCloud.features(1, 1), transformedPoint2(1));
    ASSERT_EQ(pointCloud.features(2, 1), transformedPoint2(2));
}

// This test validates the construction of base shape attributes through a derived class.
TEST_F(PointCloudGeneratorTest, SphereShape)
{ // NOLINT
    setDefaultParameters();

    // Dimensions of the sphere.
    const PM::ScalarType radius{ 1 };

    // Generate point cloud.
    const PM::DataPoints pointCloud{ PM::PointCloudGenerator::generateUniformlySampledSphere(
        radius, numberOfPoints_, translation_, orientation_) };

    // Assertions on results.
    // Number of points.
    ASSERT_EQ(pointCloud.getNbPoints(), numberOfPoints_);
    // Points correspond to volume.
    const PM::AffineTransform transformation{ PM::PointCloudGenerator::buildUpTransformation(translation_, orientation_) };
    bool isSphere{ true };
    const PM::ScalarType expectedRadiusSquared{ radius * radius };
    for (PM::DataPoints::Index i{ 0 }; i < pointCloud.features.cols(); ++i)
    {
        // Fetch point and remove transformation offset.
        const PM::StaticCoordVector point(pointCloud.features(0), pointCloud.features(1), pointCloud.features(2));
        const PM::StaticCoordVector centeredPoint{ transformation.inverse() * point };

        // Check whether the point lies inside the volume.
        const PM::ScalarType computedRadiusSquared{ centeredPoint(0) * centeredPoint(0) + centeredPoint(1) * centeredPoint(1)
                                                    + centeredPoint(2) * centeredPoint(2) };
        // For the point to belong to a sphere, its radius from the center must be within the expected margin.
        if (computedRadiusSquared > expectedRadiusSquared + kEpsilonError_)
        {
            isSphere = false;
            break;
        }
    }
    ASSERT_TRUE(isSphere);
    ASSERT_TRUE(checkNormalsPointOutwards(pointCloud, translation_));
    if (saveTestDataToDisk)
    {
        dumpTestArtifacts("Sphere", pointCloud);
    }
}

TEST_F(PointCloudGeneratorTest, CylinderShape)
{ // NOLINT
    setDefaultParameters();

    // Dimensions of the cylinder.
    const PM::ScalarType radius{ 1 };
    const PM::ScalarType height{ 2 };

    // Generate point cloud.
    const PM::DataPoints pointCloud{ PM::PointCloudGenerator::generateUniformlySampledCylinder(
        radius, height, numberOfPoints_, translation_, orientation_) };

    // Assertions on results.
    // Number of points.
    ASSERT_EQ(pointCloud.getNbPoints(), numberOfPoints_);
    // Points correspond to volume.
    const PM::AffineTransform transformation{ PM::PointCloudGenerator::buildUpTransformation(translation_, orientation_) };
    bool isCylinder{ true };
    const PM::ScalarType expectedRadiusSquared = radius * radius;
    for (PM::DataPoints::Index i{ 0 }; i < pointCloud.features.cols(); ++i)
    {
        // Fetch point and remove transformation offset.
        const PM::StaticCoordVector point(pointCloud.features(0), pointCloud.features(1), pointCloud.features(2));
        const PM::StaticCoordVector centeredPoint{ transformation.inverse() * point };

        // Check whether the point lies inside the volume.
        const PM::ScalarType computedRadiusSquared{ centeredPoint(0) * centeredPoint(0) + centeredPoint(1) * centeredPoint(1) };
        const PM::ScalarType computedHeight{ std::abs(centeredPoint(2)) };
        // For the point to belong to a cylinder, its 2D section (circle) must have the right radius, and its height must be within the
        // expected margin.
        if (computedRadiusSquared > expectedRadiusSquared + kEpsilonError_ || computedHeight > height + kEpsilonError_)
        {
            isCylinder = false;
            break;
        }
    }
    ASSERT_TRUE(isCylinder);
    ASSERT_TRUE(checkNormalsPointOutwards(pointCloud, translation_));
    if (saveTestDataToDisk)
    {
        dumpTestArtifacts("Cylinder", pointCloud);
    }
}

TEST_F(PointCloudGeneratorTest, BoxShape)
{ // NOLINT
    setDefaultParameters();

    // Dimensions of the box.
    const PM::ScalarType length{ 1 };
    const PM::ScalarType width{ 2 };
    const PM::ScalarType height{ 5 };

    // Generate point cloud.
    const PM::DataPoints pointCloud{ PM::PointCloudGenerator::generateUniformlySampledBox(
        length, width, height, numberOfPoints_, translation_, orientation_) };

    // Assertions on results.
    // Number of points.
    ASSERT_EQ(pointCloud.getNbPoints(), numberOfPoints_);
    // Points correspond to volume.
    const PM::AffineTransform transformation{ PM::PointCloudGenerator::buildUpTransformation(translation_, orientation_) };
    bool isCube{ true };
    for (PM::DataPoints::Index i{ 0 }; i < pointCloud.features.cols(); ++i)
    {
        // Fetch point and remove transformation offset.
        const PM::StaticCoordVector point(pointCloud.features(0), pointCloud.features(1), pointCloud.features(2));
        const PM::StaticCoordVector centeredPoint{ transformation.inverse() * point };

        // Check whether the point lies inside the volume.
        if (std::abs(centeredPoint(0)) > 0.5f * length + kEpsilonError_ || std::abs(centeredPoint(1)) > 0.5f * width + kEpsilonError_
            || std::abs(centeredPoint(2)) > 0.5f * height + kEpsilonError_)
        {
            isCube = false;
            break;
        }
    }
    ASSERT_TRUE(isCube);
    ASSERT_TRUE(checkNormalsPointOutwards(pointCloud, translation_));
    if (saveTestDataToDisk)
    {
        dumpTestArtifacts("Box", pointCloud);
    }
}
