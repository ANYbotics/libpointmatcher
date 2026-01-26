#include "../utest.h"
#include <ciso646>
#include <cmath>

using namespace std;
using namespace PointMatcherSupport;

//---------------------------
// DataFilter modules
//---------------------------

// Utility classes
class DataFilterTest : public IcpHelper
{
public:
    // Will be called for every tests
    virtual void SetUp()
    {
        icp.setDefault();
        // Uncomment for console outputs
        //setLogger(PM::get().LoggerRegistrar.create("FileLogger"));

        // We'll test the filters on reading point cloud
        icp.readingDataPointsFilters.clear();
    }

    // Will be called for every tests
    virtual void TearDown() {}

    void addFilter(string name, PM::Parameters params)
    {
        std::shared_ptr<PM::DataPointsFilter> testedDataPointFilter = PM::get().DataPointsFilterRegistrar.create(name, params);

        icp.readingDataPointsFilters.push_back(testedDataPointFilter);
    }

    void addFilter(string name)
    {
        std::shared_ptr<PM::DataPointsFilter> testedDataPointFilter = PM::get().DataPointsFilterRegistrar.create(name);

        icp.readingDataPointsFilters.push_back(testedDataPointFilter);
    }

    DP generateRandomDataPoints(int nbPoints = 100)
    {
        const int dimFeatures = 4;
        const int dimDescriptors = 3;
        const int dimTime = 2;

        PM::Matrix randFeat = PM::Matrix::Random(dimFeatures, nbPoints);
        DP::Labels featLabels;
        featLabels.push_back(DP::Label("x", 1));
        featLabels.push_back(DP::Label("y", 1));
        featLabels.push_back(DP::Label("z", 1));
        featLabels.push_back(DP::Label("pad", 1));

        PM::Matrix randDesc = PM::Matrix::Random(dimDescriptors, nbPoints);
        DP::Labels descLabels;
        descLabels.push_back(DP::Label("dummyDesc", 3));

        PM::Int64Matrix randTimes = PM::Int64Matrix::Random(dimTime, nbPoints);
        DP::Labels timeLabels;
        timeLabels.push_back(DP::Label("dummyTime", 2));

        // Construct the point cloud from the generated matrices
        DP pointCloud = DP(randFeat, featLabels, randDesc, descLabels, randTimes, timeLabels);

        return pointCloud;
    }
};

TEST_F(DataFilterTest, IdentityDataPointsFilter)
{
    // build test cloud
    DP ref2DCopy(ref2D);

    // apply and checked
    addFilter("IdentityDataPointsFilter");
    icp.readingDataPointsFilters.apply(ref2DCopy);
    EXPECT_TRUE(ref2D == ref2DCopy);
}

TEST_F(DataFilterTest, RemoveNaNDataPointsFilter)
{
    // build test cloud
    DP ref2DCopy(ref2D);
    int goodCount(0);
    const NumericType nan(std::numeric_limits<NumericType>::quiet_NaN());
    for (int i(0); i < ref2DCopy.features.cols(); ++i)
    {
        if (rand() % 3 == 0)
        {
            ref2DCopy.features(rand() % ref2DCopy.features.rows(), i) = nan;
        }
        else
            ++goodCount;
    }

    // apply and checked
    addFilter("RemoveNaNDataPointsFilter");
    icp.readingDataPointsFilters.apply(ref2DCopy);
    EXPECT_TRUE(ref2DCopy.features.cols() == goodCount);
}

TEST_F(DataFilterTest, MaxDistDataPointsFilter)
{
    // Max dist has been selected to not affect the points
    params = PM::Parameters();
    params["dim"] = "0";
    params["maxDist"] = toParam(6.0);

    // Filter on x axis
    params["dim"] = "0";
    icp.readingDataPointsFilters.clear();
    addFilter("MaxDistDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();

    // Filter on y axis
    params["dim"] = "1";
    icp.readingDataPointsFilters.clear();
    addFilter("MaxDistDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();

    // Filter on z axis (not existing)
    params["dim"] = "2";
    icp.readingDataPointsFilters.clear();
    addFilter("MaxDistDataPointsFilter", params);
    EXPECT_ANY_THROW(validate2dTransformation());
    validate3dTransformation();

    // Filter on a radius
    params["dim"] = "-1";
    icp.readingDataPointsFilters.clear();
    addFilter("MaxDistDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();

    // Parameter outside valid range
    params["dim"] = "3";
    //TODO: specify the exception, move that to GenericTest
    EXPECT_ANY_THROW(addFilter("MaxDistDataPointsFilter", params));
}

TEST_F(DataFilterTest, MinDistDataPointsFilter)
{
    // Min dist has been selected to not affect the points too much
    params = PM::Parameters();
    params["dim"] = "0";
    params["minDist"] = toParam(0.05);

    // Filter on x axis
    params["dim"] = "0";
    icp.readingDataPointsFilters.clear();
    addFilter("MinDistDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();

    // Filter on y axis
    params["dim"] = "1";
    icp.readingDataPointsFilters.clear();
    addFilter("MinDistDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();

    //TODO: move that to specific 2D test
    // Filter on z axis (not existing)
    params["dim"] = "2";
    icp.readingDataPointsFilters.clear();
    addFilter("MinDistDataPointsFilter", params);
    EXPECT_ANY_THROW(validate2dTransformation());
    validate3dTransformation();

    // Filter on a radius
    params["dim"] = "-1";
    icp.readingDataPointsFilters.clear();
    addFilter("MinDistDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();
}


TEST_F(DataFilterTest, SurfaceNormalDataPointsFilter)
{
    // This filter create descriptor, so parameters should'nt impact results
    params = PM::Parameters();
    params["knn"] = "5";
    params["epsilon"] = "0.1";
    params["keepNormals"] = "1";
    params["keepDensities"] = "1";
    params["keepEigenValues"] = "1";
    params["keepEigenVectors"] = "1";
    params["keepMatchedIds"] = "1";
    // FIXME: the parameter keepMatchedIds seems to do nothing...

    addFilter("SurfaceNormalDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();

    // TODO: standardize how filter are tested:
    // 1- impact on number of points
    // 2- impact on descriptors
    // 3- impact on ICP (that's what we test now)
}

TEST_F(DataFilterTest, MaxDensityDataPointsFilter)
{
    // Ratio has been selected to not affect the points too much
    vector<double> ratio = { 100, 1000, 5000 };

    for (unsigned i = 0; i < ratio.size(); i++)
    {
        icp.readingDataPointsFilters.clear();
        params = PM::Parameters();
        params["knn"] = "5";
        params["epsilon"] = "0.1";
        params["keepNormals"] = "0";
        params["keepDensities"] = "1";
        params["keepEigenValues"] = "0";
        params["keepEigenVectors"] = "0";
        params["keepMatchedIds"] = "0";

        addFilter("SurfaceNormalDataPointsFilter", params);

        params = PM::Parameters();
        params["maxDensity"] = toParam(ratio[i]);

        addFilter("MaxDensityDataPointsFilter", params);

        // FIXME BUG: the density in 2D is not well computed
        //validate2dTransformation();

        //double nbInitPts = data2D.features.cols();
        //double nbRemainingPts = icp.getPrefilteredReadingPtsCount();
        //EXPECT_TRUE(nbRemainingPts < nbInitPts);

        validate3dTransformation();

        double nbInitPts = data3D.features.cols();
        double nbRemainingPts = icp.getPrefilteredReadingPtsCount();
        EXPECT_TRUE(nbRemainingPts < nbInitPts);
    }
}

TEST_F(DataFilterTest, SamplingSurfaceNormalDataPointsFilter)
{
    // This filter create descriptor AND subsample
    params = PM::Parameters();
    params["knn"] = "5";
    params["averageExistingDescriptors"] = "1";
    params["keepNormals"] = "1";
    params["keepDensities"] = "1";
    params["keepEigenValues"] = "1";
    params["keepEigenVectors"] = "1";

    addFilter("SamplingSurfaceNormalDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();
}


TEST_F(DataFilterTest, OrientNormalsDataPointsFilter)
{
    // Used to create normal for reading point cloud
    std::shared_ptr<PM::DataPointsFilter> extraDataPointFilter;
    extraDataPointFilter = PM::get().DataPointsFilterRegistrar.create("SurfaceNormalDataPointsFilter");
    icp.readingDataPointsFilters.push_back(extraDataPointFilter);
    addFilter("ObservationDirectionDataPointsFilter");
    addFilter("OrientNormalsDataPointsFilter", { { "towardCenter", toParam(false) } });
    validate2dTransformation();
    validate3dTransformation();
}


TEST_F(DataFilterTest, RandomSamplingDataPointsFilter)
{
    for (const double prob : { 0.8, 0.85, 0.9, 0.95 })
    {
        for (const unsigned int samplingMethod : { 0, 1 })
        {
            // Try to avoid too low value for the reduction to avoid under sampling
            params = PM::Parameters();
            params["prob"] = toParam(prob);
            params["randomSamplingMethod"] = toParam(samplingMethod);

            icp.readingDataPointsFilters.clear();
            addFilter("RandomSamplingDataPointsFilter", params);
            validate2dTransformation();
            validate3dTransformation();
        }
    }
}


TEST_F(DataFilterTest, MaxPointCountDataPointsFilter)
{
    DP cloud = ref3D;

    const size_t maxCount = 1000;

    params = PM::Parameters();
    params["seed"] = "42";
    params["maxCount"] = toParam(maxCount);

    std::shared_ptr<PM::DataPointsFilter> maxPtsFilter =
        PM::get().DataPointsFilterRegistrar.create("MaxPointCountDataPointsFilter", params);

    DP filteredCloud = maxPtsFilter->filter(cloud);

    //Check number of points
    EXPECT_GT(cloud.getNbPoints(), filteredCloud.getNbPoints());
    EXPECT_EQ(cloud.getDescriptorDim(), filteredCloud.getDescriptorDim());
    EXPECT_EQ(cloud.getTimeDim(), filteredCloud.getTimeDim());

    EXPECT_EQ(filteredCloud.getNbPoints(), maxCount);

    //Same seed should result same filtered cloud
    DP filteredCloud2 = maxPtsFilter->filter(cloud);

    EXPECT_TRUE(filteredCloud == filteredCloud2);

    //Different seeds should not result same filtered cloud but same number
    params.clear();
    params["seed"] = "1";
    params["maxCount"] = toParam(maxCount);

    std::shared_ptr<PM::DataPointsFilter> maxPtsFilter2 =
        PM::get().DataPointsFilterRegistrar.create("MaxPointCountDataPointsFilter", params);

    DP filteredCloud3 = maxPtsFilter2->filter(cloud);

    EXPECT_FALSE(filteredCloud3 == filteredCloud2);

    EXPECT_EQ(filteredCloud3.getNbPoints(), maxCount);

    EXPECT_EQ(filteredCloud3.getNbPoints(), filteredCloud2.getNbPoints());
    EXPECT_EQ(filteredCloud3.getDescriptorDim(), filteredCloud2.getDescriptorDim());
    EXPECT_EQ(filteredCloud3.getTimeDim(), filteredCloud2.getTimeDim());

    //Validate transformation
    icp.readingDataPointsFilters.clear();
    addFilter("MaxPointCountDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();
}

TEST_F(DataFilterTest, OctreeGridDataPointsFilter)
{
    const unsigned int nbPts = 60000;
    const DP cloud = generateRandomDataPoints(nbPts);
    params = PM::Parameters();

    std::shared_ptr<PM::DataPointsFilter> octreeFilter;

    for (const int samplingMethod : { 0 })
        for (const size_t maxData : { 1, 5 })
            for (const NumericType maxSize : { 0., 0.05 })
            {
                params.clear();
                params["maxPointByNode"] = toParam(maxData);
                params["maxSizeByNode"] = toParam(maxSize);
                params["samplingMethod"] = toParam(samplingMethod);
                params["buildParallel"] = "1";
                params["centerAtOrigin"] = "1";

                octreeFilter = PM::get().DataPointsFilterRegistrar.create("OctreeGridDataPointsFilter", params);

                const DP filteredCloud = octreeFilter->filter(cloud);

                if (maxData == 1 and maxSize == 0.)
                {
                    // 1/pts by octants + validate parallel build
                    // the number of point should not change
                    // the sampling methods should not change anything
                    //Check number of points
                    EXPECT_EQ(cloud.getNbPoints(), filteredCloud.getNbPoints());
                    EXPECT_EQ(cloud.getDescriptorDim(), filteredCloud.getDescriptorDim());
                    EXPECT_EQ(cloud.getTimeDim(), filteredCloud.getTimeDim());

                    EXPECT_EQ(filteredCloud.getNbPoints(), nbPts);
                }
                else
                {
                    //Check number of points
                    EXPECT_GT(cloud.getNbPoints(), filteredCloud.getNbPoints());
                }
                //Validate transformation
                icp.readingDataPointsFilters.clear();
                addFilter("OctreeGridDataPointsFilter", params);
                validate2dTransformation();
                validate3dTransformation();
            }
}

TEST_F(DataFilterTest, NormalSpaceDataPointsFilter)
{
    const size_t nbPts = 60000;
    DP cloud = generateRandomDataPoints(nbPts);
    params = PM::Parameters();

    //const size_t nbPts2D = ref2D.getNbPoints();
    const size_t nbPts3D = ref3D.getNbPoints();

    std::shared_ptr<PM::DataPointsFilter> nssFilter;

    //Compute normals
    auto paramsNorm = PM::Parameters();
    paramsNorm["knn"] = "5";
    paramsNorm["epsilon"] = "0.1";
    paramsNorm["keepNormals"] = "1";
    std::shared_ptr<PM::DataPointsFilter> normalFilter =
        PM::get().DataPointsFilterRegistrar.create("SurfaceNormalDataPointsFilter", paramsNorm);

    normalFilter->inPlaceFilter(cloud);

    //Evaluate filter
    std::vector<size_t> samples = { /* 2*nbPts2D/3, nbPts2D,*/ 1500, 5000, nbPts, nbPts3D };
    for (const NumericType epsilon : { M_PI / 6., M_PI / 32., M_PI / 64. })
        for (const size_t nbSample : samples)
        {
            icp.readingDataPointsFilters.clear();

            params.clear();
            params["epsilon"] = toParam(epsilon);
            params["nbSample"] = toParam(nbSample);

            nssFilter = PM::get().DataPointsFilterRegistrar.create("NormalSpaceDataPointsFilter", params);

            addFilter("SurfaceNormalDataPointsFilter", paramsNorm);
            addFilter("NormalSpaceDataPointsFilter", params);

            const DP filteredCloud = nssFilter->filter(cloud);

            /*
			if(nbSample <= nbPts2D)
			{
				validate2dTransformation();
				EXPECT_LE(filteredCloud.getNbPoints(), nbPts2D);
				continue;
			}
			else if (nbSample == nbPts3D)
			{
				EXPECT_EQ(filteredCloud.getNbPoints(), nbPts3D);
			}
			else */
            if (nbSample == nbPts)
            {
                //Check number of points
                EXPECT_EQ(cloud.getNbPoints(), filteredCloud.getNbPoints());
                EXPECT_EQ(cloud.getDescriptorDim(), filteredCloud.getDescriptorDim());
                EXPECT_EQ(cloud.getTimeDim(), filteredCloud.getTimeDim());

                EXPECT_EQ(filteredCloud.getNbPoints(), nbPts);
            }

            validate3dTransformation();
            EXPECT_GE(cloud.getNbPoints(), filteredCloud.getNbPoints());
        }
}


TEST_F(DataFilterTest, DistanceLimitDataPointsFilter)
{
    params = PM::Parameters();
    params["dim"] = "0";
    params["dist"] = toParam(6.0);
    params["removeInside"] = "0";

    // Filter on x axis
    params["dim"] = "0";
    icp.readingDataPointsFilters.clear();
    addFilter("DistanceLimitDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();

    // Filter on y axis
    params["dim"] = "1";
    icp.readingDataPointsFilters.clear();
    addFilter("DistanceLimitDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();

    // Filter on z axis (not existing)
    params["dim"] = "2";
    icp.readingDataPointsFilters.clear();
    addFilter("DistanceLimitDataPointsFilter", params);
    EXPECT_ANY_THROW(validate2dTransformation());
    validate3dTransformation();

    // Filter on a radius
    params["dim"] = "-1";
    icp.readingDataPointsFilters.clear();
    addFilter("DistanceLimitDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();

    // Parameter outside valid range
    params["dim"] = "3";
    //TODO: specify the exception, move that to GenericTest
    EXPECT_ANY_THROW(addFilter("DistanceLimitDataPointsFilter", params));


    params = PM::Parameters();
    params["dim"] = "0";
    params["dist"] = toParam(0.05);
    params["removeInside"] = "1";

    // Filter on x axis
    params["dim"] = "0";
    icp.readingDataPointsFilters.clear();
    addFilter("DistanceLimitDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();

    // Filter on y axis
    params["dim"] = "1";
    icp.readingDataPointsFilters.clear();
    addFilter("DistanceLimitDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();

    //TODO: move that to specific 2D test
    // Filter on z axis (not existing)
    params["dim"] = "2";
    icp.readingDataPointsFilters.clear();
    addFilter("DistanceLimitDataPointsFilter", params);
    EXPECT_ANY_THROW(validate2dTransformation());
    validate3dTransformation();

    // Filter on a radius
    params["dim"] = "-1";
    icp.readingDataPointsFilters.clear();
    addFilter("DistanceLimitDataPointsFilter", params);
    validate2dTransformation();
    validate3dTransformation();
}

TEST_F(DataFilterTest, SameFilterInstanceTwice)
{
    params = PM::Parameters();

    std::shared_ptr<PM::DataPointsFilter> df = PM::get().DataPointsFilterRegistrar.create("MaxPointCountDataPointsFilter", params);

    icp.referenceDataPointsFilters.push_back(df);
    icp.readingDataPointsFilters.push_back(df);
}
