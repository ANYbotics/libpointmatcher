#include "../utest.h"

using namespace std;
using namespace PointMatcherSupport;

//---------------------------
// Matcher modules
//---------------------------

// Utility classes
class MatcherTest: public IcpHelper
{

public:

	std::shared_ptr<PM::Matcher> testedMatcher;

	// Will be called for every tests
	virtual void SetUp()
	{
		icp.setDefault();
		// Uncomment for consol outputs
		//setLogger(PM::get().LoggerRegistrar.create("FileLogger"));
	}

	// Will be called for every tests
	virtual void TearDown(){}

	void addFilter(string name, PM::Parameters params)
	{
		testedMatcher = 
			PM::get().MatcherRegistrar.create(name, params);
		icp.matcher = testedMatcher;
	}

};

TEST_F(MatcherTest, MirrorMatcher)
{
    // Generate test data.
    const PM::StaticCoordVector translation{ PM::StaticCoordVector::Zero() };
    const PM::Quaternion orientation{ PM::Quaternion::Identity() };
    const PM::DataPoints::Index numberOfPoints{ 10000 };
    const PM::ScalarType radius{ 10 };
    const PM::DataPoints pointCloud{ PM::PointCloudGenerator::generateUniformlySampledSphere(
        radius, numberOfPoints, translation, orientation) };

    // Init matcher.
    auto matcher = PM::get().MatcherRegistrar.create("MirrorMatcher", PM::Parameters());
    matcher->init(pointCloud);

    // Find matches.
    const auto matches(matcher->findClosests(pointCloud));

    // Validate matches.
    bool areMatchesPerfect{ true };
    for (PM::DataPoints::Index i = 0; i < numberOfPoints; ++i)
    {
        if ((matches.dists(0, i) != 0) || (matches.ids(0, i) != i))
        {
            areMatchesPerfect = false;
			break;
        }
    }
    ASSERT_TRUE(areMatchesPerfect);
}

TEST_F(MatcherTest, KDTreeMatcher)
{
	vector<unsigned> knn = {1, 2, 3};
	vector<double> epsilon = {0.0, 0.2};
	vector<double> maxDist = {1.0, 0.5};

	for(unsigned i=0; i < knn.size(); i++)
	{
		for(unsigned j=0; j < epsilon.size(); j++)
		{
			for(unsigned k=0; k < maxDist.size(); k++)
			{
				params = PM::Parameters();
				params["knn"] = toParam(knn[i]); // remove end parenthesis for bug
				params["epsilon"] = toParam(epsilon[j]);
				params["maxDist"] = toParam(maxDist[k]);
				
			
				addFilter("KDTreeMatcher", params);
				validate2dTransformation();
				validate3dTransformation();
			}
		}
	}
}