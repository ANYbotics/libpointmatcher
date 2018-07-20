^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libpointmatcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.4 (2015-07-20)
------------------
* Merge branch 'master' into debian
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* added message when force 2D is used
* update example for C++11
* added observationDirections to be recovered as a vector when point clouds are loaded from files
* Merge pull request `#261 <https://github.com/samuelba/libpointmatcher/issues/261>`_ from norlab-ulaval/fix/nss
  Quick fix for 2D test case in NormalSpace filter
* [fix] update documentation
* [fix] remove 2D case in utest
* [fix] add check for pointcloud dimension (only 3D support)
* Merge pull request `#259 <https://github.com/samuelba/libpointmatcher/issues/259>`_ from norlab-ulaval/doc/datafilters
  Update documentation of datafilters
* [doc] emphasize deprecated for voxel grid filter
* [doc] minor fix for octree grid filter doc
* [doc] minor fix for covs filter doc
* [doc] add CovS filter documentation
* [doc] add NSS filter documentation
* [doc] minor fix for octree grid filter doc
* [doc] add Octree Grid Filter documentation
* [filter] add torque normalization methods
* [utest] remove 2D case from test (filter only for 3D pts)
* [fix] move assertion for 3D pts at the beginning of the function
* [fix] fix typos
* [utest] add utest to validate filter
* [biblio] add reference to bibliography
* [filter] register the filter
* [filter] add Covariance Sampling filter
* Merge pull request `#257 <https://github.com/samuelba/libpointmatcher/issues/257>`_ from norlab-ulaval/feature/octreegrid
  Add a new sampling method for Octree Grid filter based on "medoid"
* [biblio] add reference to bibliography
* [utest] update utest
* [filter] add a new sampling method based on medoid
* [utest] fix utest for 3D case with not enought pts
* [filter] remove magic number
* [filter] fix phi calculation
* [filter] fix bounds for espilon parameters + clean code
* Merge branch 'master' into feature/nss
* Merge pull request `#255 <https://github.com/samuelba/libpointmatcher/issues/255>`_ from norlab-ulaval/feature/octree
  Major revision of the Octree class
* [utest] add unit test to validate NSS filter
* [filter] register the filter
* [filter] add Normal Space Sampling filter
* [fix] clean utest, pow function and clean octree code
* [api] move swapCols function to the DataPoints class
* [utest] fix parameter values due to fail in 2D case
* [octree] fix bounding box not assigned in leaf case
* [octree] add getters for boundingbox values
* [filter] fix index in loop for centroid calculation
* [filter] fix the array intialization causing errors
* [utils] add link in comment for the pow metafunction
* [octree] add missing get for futures + deduplicate code into a unique lambda
* [octree] rename + correct to new indexes + simplify helper struct into OctreeHelper
* [octree] change indexes order (xzy to xyz), no more special case for 3D
* [utils] rename utils namespace into PointMatcherSupport
* [utils] rewrite pow metafunction more concisely
* Merge branch 'master' into feature/octree
* [filter] add support for 2D case with quadtree
* [utest] enable 2D test case for Quadtree
* [octree] fix typo
* Merge pull request `#210 <https://github.com/samuelba/libpointmatcher/issues/210>`_ from lorenwel/fix/kdtree_recompute
  Fixed rebuilding kdtree every registration
* Merge branch 'master' into fix/kdtree_recompute
* [octree] change threads by std::async and std::launch::async as suggested
* [utest] clean unit test for OctreeGridFilter
* [filter] remove build method parameter since the octree do not differentiate them anymore
* [octree] rewrite the octree class to avoid duplication (take into account reviews in `#250 <https://github.com/samuelba/libpointmatcher/issues/250>`_)
* [utils] add metafunction to compute pow at compile time
* Merge pull request `#250 <https://github.com/samuelba/libpointmatcher/issues/250>`_ from norlab-ulaval/feature/octree
  Add a new datafilter based on octree
* [filter] fix non-const reference initialization
* Merge branch 'master' into feature/octree
* [filter] correct centroid calculation + modify swapCols function due to Eigen specifics
* [octree] fix typo in move constructor
* [filter] assert 3D points before octree build + add missing finalize calls
* [filter] fix bad_lexical_cast
* [utest] add utest for OctreeGridFilter
* [utest] set nbPts as parameter of generate DataPoints function
* [cmake] add OctreeGridFilter entry
* [filter] add OctreeGridFilter to registrar
* [filter] add filter implementation
* [octree] move octree.cpp into octree.hpp due to link issue
* [filter] add Samplers implementations
* [filter] complete Samplers interfaces
* [octree] remove inline qualifiers
* [octree] fix typo
* [octree] change parallel build as parameters + minors optimization
* add Octree grid filter interface
* Merge branch 'master' into fix/kdtree_recompute
* Changed deprecation warning.
* Added deprecation warnings.
* Merge pull request `#206 <https://github.com/samuelba/libpointmatcher/issues/206>`_ from ethz-asl/fix/clangCompilation
  Fix compilation with clang
* add octree class
* Merge branch 'master' into fix/kdtree_recompute
* Merge branch 'master' into fix/clangCompilation
* Merge pull request `#248 <https://github.com/samuelba/libpointmatcher/issues/248>`_ from norlab-ulaval/doc/dataFilters
  Update documentation of DataFilters dev
* Merge branch 'master' into doc/dataFilters
* update DataFilter dev documentation due to Data Filters refactoring (see `#246 <https://github.com/samuelba/libpointmatcher/issues/246>`_)
* minors fix; should resolve issue `#245 <https://github.com/samuelba/libpointmatcher/issues/245>`_
* fix MaxPointCountDataPointsFilter
* add MaxPointCountDataPointsFilter unit test
* Merge pull request `#237 <https://github.com/samuelba/libpointmatcher/issues/237>`_ from svenevs/typo
  fix two typos
* Merge branch 'master' into typo
* Merge pull request `#246 <https://github.com/samuelba/libpointmatcher/issues/246>`_ from norlab-ulaval/refactoring/dataFilters
  Refactoring of the Data Filters
* Merge branch 'master' into refactoring/dataFilters
* Merge branch 'master' into typo
* ignore jupyter checkpoints
* Merge branch 'master' into typo
* added extension .cmake to UseDoxygen to comply with cmake new standards
* add license
* clean files
* refactor GestaltDataPointsFilter
* refactor ElipsoidsDataPointsFilter
* refactor CutAtDescriptorThresholdDataPointsFilter
* refactor VoxelGridDataPointsFilter
* refactor ObservationDirectionDataPointsFilter
* refactor SimpleSensorNoiseDataPointsFilter
* refactor ShadowDataPointsFilter
* refactor FixStepSamplingDataPointsFilter
* refactor MaxPointCountDataPointsFilter but may need a fix (see issue `#245 <https://github.com/samuelba/libpointmatcher/issues/245>`_)
* refactor RandomSamplingDataPointsFilter
* refactor IncidenceAngleDataPointsFilter
* refactor OrientNormalsDataPointsFilter
* refactor SamplingSurfaceNormalDataPointsFilter
* refactor SurfaceNormalDataPointsFilter
* add utils header with often used functions
* refactor MaxDensityDataPointsFilter
* refactor MaxQuantileOnAxisDataPointsFilter
* refactor BoundingBoxDataPointsFilter
* refactor MinDistDataPointsFilter
* refactor MaxDistDataPointsFilter
* refactor RemoveNaNDataPointsFilter
* refactor IdentityDataPointsFilter
* add IdentityDataPointsFilter utest
* Merge pull request `#244 <https://github.com/samuelba/libpointmatcher/issues/244>`_ from norlab-ulaval/fix/datafilters
  Fix/datafilters : clean Data Filters before reorganizing them
* fix bad_lexical_cast in utest VoxelGridDataPointsFilter
* minors fix in DataPointfilters (add const qualifiers, post to prefix, bracket style, etc..)
* Merge pull request `#243 <https://github.com/samuelba/libpointmatcher/issues/243>`_ from norlab-ulaval/fix/doc
  Fix some errors in DataFilters documentation
* ensure non zero value for voxel sizes in doc of VoxelGridDataPointsfilter
* remove unvalid link from Surface Normal Filter documentation
* correct range value for voxel sizes in doc of VoxelGridDataPointsfilter
* Update index.md
* fix two typos
* Merge pull request `#235 <https://github.com/samuelba/libpointmatcher/issues/235>`_ from svenevs/cmake_no_grep
  remove `grep` dependency, parse version with CMake
* remove `grep` dependency, parse version with CMake
* Merge pull request `#232 <https://github.com/samuelba/libpointmatcher/issues/232>`_ from BenBallard/PassByConstReference
  Changed pass by const value to pass by const reference to remove the extra copy.
* Changed pass by const value to pass by const reference
* Merge pull request `#230 <https://github.com/samuelba/libpointmatcher/issues/230>`_ from ethz-asl/fix/dontUseInvalidIndices
  Avoid using invalid matches
* Inserting spaces after touched if and for
* Use InvalidId and InvalidDist to avoid using invalid matches
* Introduced Matcher::InvalidId and InvalidDist
* Set weight to 0 if the index is invalid.
* Prevent using invalid matches in SurfaceNormalOutlierFilter::compute
* Merge pull request `#220 <https://github.com/samuelba/libpointmatcher/issues/220>`_ from tomifischer/point-to-plane-with-covariance
  Point to Plane with Covariance Error Minimizer
* fix whitespace
* updating master
* dont know where these came from
* .
* added check to skip null vectors in covariance calculation
* fixed sensor covariance matrix size
* Added covariance calculation for 2D case
* Merge pull request `#217 <https://github.com/samuelba/libpointmatcher/issues/217>`_ from artivis/handle_reflexion
  handle reflexion in PointToPointWithCovErrorMinimizer
* Merge branch 'master' into handle_reflexion
* Merge pull request `#218 <https://github.com/samuelba/libpointmatcher/issues/218>`_ from ethz-asl/fix/nabo_install
  Fixed usage of the LIBNABO_INSTALL_DIR variable
* Fixed usage of the LIBNABO_INSTALL_DIR variable
  This is how one can amend the prefix list for find_package
* handle reflexion in PointToPointWithCovErrorMinimizer
* Merge branch 'master' into fix/kdtree_recompute
* Update UnitTestDev.md
* Update Configuration.md
* Merge pull request `#208 <https://github.com/samuelba/libpointmatcher/issues/208>`_ from davidlandry93/master
  Remove duplicated code in PointToPlaneErrorMinimizer
* Merge branch 'master' into master
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* Addressed more comments from the pr
* Merge remote-tracking branch 'ethz/master'
* Added a forward def of error minimizers for compatibility
* ajusted threshold
* Merge pull request `#212 <https://github.com/samuelba/libpointmatcher/issues/212>`_ from ethz-asl/fix/doc
  changed links to the list of tutorials
* changed links to the list of tutorials
* Addressed comments
* Fixed rebuilding kdtree every registration
* Indentation type
* Removed useless methods in ptpwithcov
* Now have a shared constructor
* Moved point to plane out of the bloat as well
* Added unit tests for with cov
* Moved PointToPlaneWithCovErrorMinimizer outside of the monster
* Merge pull request `#1 <https://github.com/samuelba/libpointmatcher/issues/1>`_ from ethz-asl/master
  Bring back changes from master
* Fix compilation with clang
* Merge pull request `#204 <https://github.com/samuelba/libpointmatcher/issues/204>`_ from ethz-asl/feature/useC++11
  Enable c++11 for libpointmatcher itself (addressing `#202 <https://github.com/samuelba/libpointmatcher/issues/202>`_)
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* add extra verification in VoxelGrid to catch NaNs
* add extra test for VoxelGrid
* Enable c++11 for libpointmatcher itself (addressing `#202 <https://github.com/samuelba/libpointmatcher/issues/202>`_)
* Merge pull request `#203 <https://github.com/samuelba/libpointmatcher/issues/203>`_ from ethz-asl/fix/moveNaboIncludeToWhereItIsNeeded
  Moved include of nabo to MatchersImpl.h, where it is actually required
* Moved include of nabo to MatchersImpl.h, where it is actually required
* Merge pull request `#201 <https://github.com/samuelba/libpointmatcher/issues/201>`_ from bryant1410/master
  Fix broken headings in Markdown files
* Fix broken Markdown headings
* Update README.md
* Merge pull request `#200 <https://github.com/samuelba/libpointmatcher/issues/200>`_ from tushar-dadlani/patch-1
  Fix markdown syntax error
* Fix markdown syntax error
* update changelog
* Merge remote-tracking branch 'upstream/master' into debian
* Update Datafilters.md
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* correct bug with VoxelGrid and times field. Add print outputs for labels
* add an example yaml file for minimization with scale
* Update ApplyingDatafilters.md
* Update ApplyingDatafilters.md
* Update Datafilters.md
* Update Datafilters.md
* add jupyter script to plot results
* add new filter and fix SurfaceNormalDataFilter
* Update index.md
* fix debian
* Update DataPointsFilterDev.md
* Update index.md
* Merge pull request `#191 <https://github.com/samuelba/libpointmatcher/issues/191>`_ from ethz-asl/fix/warnings
  clean compilations warnings
* clean compilations warnings
* Update CompilationWindows.md
* Merge pull request `#190 <https://github.com/samuelba/libpointmatcher/issues/190>`_ from ethz-asl/fix/issue185
  Fix/issue185
* remove commented functions
* finish refactoring of loadPCD()
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher into fix/issue185
* Update index.md
* update the example with a better API
* Merge pull request `#189 <https://github.com/samuelba/libpointmatcher/issues/189>`_ from ethz-asl/fix/issueWithCorrectParam
  fix problem with rotation matrix not well corrected
* fix problem with rotation matrix not well corrected
* start cleaning PLY load
* clean output for the example code pmicp
* Update ICPIntro.md
* Update icp_tutorial_cfg.yaml
* Update align_sequence.cpp
* Update icp_tutorial_empty.yaml
* Update Datafilters.md
* Merge pull request `#184 <https://github.com/samuelba/libpointmatcher/issues/184>`_ from ethz-asl/fix/issue182
  added intensity to the list of supported descriptor
* Merge pull request `#183 <https://github.com/samuelba/libpointmatcher/issues/183>`_ from ethz-asl/fix/issue181
  Fixed issue `#181 <https://github.com/samuelba/libpointmatcher/issues/181>`_.
* remove dead statement for old eigen version
* added intensity to the list of supported descriptor
* Fixed issue `#181 <https://github.com/samuelba/libpointmatcher/issues/181>`_.
* Merge pull request `#179 <https://github.com/samuelba/libpointmatcher/issues/179>`_ from ethz-asl/fix/installHeadersAgain
  Public headers get installed again into ${INSTALL_INCLUDE_DIR}/pointmatcher.
* Public headers get installed again into ${INSTALL_INCLUDE_DIR}/pointmatcher.
  (Fixed regression introduced with 1066b29d61e1f55abd93c8e3cf.)
* Update Compilation.md
* Update README.md
* better name and message for cmake
* add a cmake option to enable/disable documentation
* remove warnings from Doxygen configuration
* xMerge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* add a yaml file to simplify the tutorial
* Update ICPIntro.md
* Update Compilation.md
* add missing Functions.h to install target
* resolved bug with missing time in ErrorElements
* added unit test for ErrorElements constructor
* resolved conflict
* Merge branch 'feature/addTimeToVtkExport'
* remove the concept of sec and nsec
* improve safety checks
* added unit test for time in binary VTK
* add unit test for time in VTK
* finished implementing loading time in VTK
* finished to export time to VTK
* Merge pull request `#168 <https://github.com/samuelba/libpointmatcher/issues/168>`_ from ethz-asl/fix/someSmallValigrindMotivatedFixes
  Fix/some small valigrind motivated fixes
* implement time convertion to vtk (not tested)
* Fixed some memory leaks in the tests. This helps finding bugs with tools like valgrind
* Removed memory leak in GestaltDataPointsFilter::buildNew
* Minor improvements for PointMatcherIO<T>::loadVTK
* Fixed memory hole in PointMatcherIO<T>::loadPLY
* Fixed minor memory hole in Registrar (in case of
  InvalidParameter)
* add some extra file check and fix issue `#167 <https://github.com/samuelba/libpointmatcher/issues/167>`_
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* Merge branch 'Ellon-feature/getResidualError'
* implemente simpler interface for ErrorMinimizers
* change contructor of ErrorElements to simplify maintenance
* change info to warn level
* Merge branch 'feature/getResidualError' of https://github.com/Ellon/libpointmatcher into Ellon-feature/getResidualError
* Merge pull request `#166 <https://github.com/samuelba/libpointmatcher/issues/166>`_ from oleg-alexandrov/transform-fix
  Throw std::runtime_error
* Throw std::runtime_error
* Update ReleaseNotes.md
* Update ReleaseNotes.md
* Merge pull request `#165 <https://github.com/samuelba/libpointmatcher/issues/165>`_ from oleg-alexandrov/transform-fix
  Transformation.cpp: There is no chain, the transform must be applied once
* Transformation.cpp: There is no chain, the transform must be applied once
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* fix new outlier module
* fix conflict
* Update DataPointsFiltersImpl.cpp
* Added header for robust welsch function
* Merge pull request `#160 <https://github.com/samuelba/libpointmatcher/issues/160>`_ from ethz-asl/feature/pmicpVerbose
  Made pmicp more verbose (enabled logging to std output)
* Merge pull request `#159 <https://github.com/samuelba/libpointmatcher/issues/159>`_ from ethz-asl/fix/betterBehaviorForPointToPlainMinimizationYieldingSingularMatrix
  Fix/better behavior for point to plain minimization yielding singular matrix
* Added '--verbose' option (default false) to pmicp example. Iff verbose then the LOG_INFO_STREAM output gets passwd to stdout
* Turned ConvergenceError in solvePossiblyUnderdeterminedLinearSystem into pure warning and tried to make the condition more suitable.
* Made pmicp more verbose (enabled logging to std output)
* Improved behavior when encountering singular matrix while minimizing point to plane (addressing `#158 <https://github.com/samuelba/libpointmatcher/issues/158>`_)
* Fixed use of comma initializer in icpTest.icpSingular
* Update README.md
* [error minimizer] Add getResidualError methods
  getResidualError is a const method and makes use of getMatchedPoints, thus we
  also add a const version of getMatchedPoints that does not change
  ErrorMinimizer's lastErrorElements.
  The residual error is computed in a separated function defined in an anonymous
  namespace
* Merge pull request `#149 <https://github.com/samuelba/libpointmatcher/issues/149>`_ from ethz-asl/fix/supportSingularICPStepsInPointToPlaneErrorMinimizer
  Support singular ICP steps in PointToPlane(Cov)ErrorMinimizer
* Merge pull request `#150 <https://github.com/samuelba/libpointmatcher/issues/150>`_ from oleg-alexandrov/minor_fixes
  Minor fixes
* Clarify similarity transform
  tweak
* IO: Fix an error, the input var was not used, a member was assigned to itself instead
* Merge pull request `#132 <https://github.com/samuelba/libpointmatcher/issues/132>`_ from ethz-asl/feature/writeBinaryVTKFiles
  Support binary VTK legacy files
* Update mkdocs.yml
* Support singular ICP steps in PointToPlaneErrorMinimizer
* Merge pull request `#148 <https://github.com/samuelba/libpointmatcher/issues/148>`_ from oleg-alexandrov/doc
  Doc expansion
* Merge pull request `#147 <https://github.com/samuelba/libpointmatcher/issues/147>`_ from oleg-alexandrov/similarity
  Similarity
* clarify doc
* Clarify doc
* Expand the doc
* What if we tighten a bit the tolerance
* relax even more the tol
* revamp how the error of icp is evaluated
* Add similarity transform, with test
* Make the code  compile
* Undo some hacks to PointToPointErrorMinimizer
* fix error minimizer: use w.transpose() to get sigma
* bibliography: add Umeyama1991 for similarity error minimizer
  Note that this paper does not include the weights!
* PointToPointErrorMinimizer: use similarity transformation
  add additional DoF: scaling parameter (This breaks assumption that the
  transformation is rigid!)
* Merge pull request `#146 <https://github.com/samuelba/libpointmatcher/issues/146>`_ from oleg-alexandrov/AddPointToPointFilter
  Add yaml file testing point-to-point icp
* Update Chen91_pt2plane.yaml
* Merge pull request `#145 <https://github.com/samuelba/libpointmatcher/issues/145>`_ from oleg-alexandrov/YamlFileBugFix
  Fix for a no-longer valid yaml file
* Relax tranlation tol in unit test
* Add yaml file testing point-to-point icp
* Fix for a no-longer valid yaml file
* Improved PointMatcher<T>::DataPoints::save failure handling. Especially throw on binary = true for formats that don't support it.
* Support reading of binary VTK legacy files, too.
* Support writing of binary VTK legacy files
* correct missing copy of the time field in the rigid transformation
* ajust the usage text
* change how the overlap is computed for pointToPlane
* Update ApplicationsAndPub.md
* address issue `#112 <https://github.com/samuelba/libpointmatcher/issues/112>`_ and `#139 <https://github.com/samuelba/libpointmatcher/issues/139>`_
* soved issue `#142 <https://github.com/samuelba/libpointmatcher/issues/142>`_
* remove a unit test, highlight some problems with Eigen
* Merge pull request `#138 <https://github.com/samuelba/libpointmatcher/issues/138>`_ from vanurag/fix/nan_crash
  Calculating mean using filtered data to avoid NaNs
* calculating mean using filtered data to avoid NaNs
* Update CompilationWindows.md
* Merge pull request `#135 <https://github.com/samuelba/libpointmatcher/issues/135>`_ from bkueng/weighted_error_minimizer
  Weighted error minimizer
* ErrorMinimizer: avoid replicate(), ignore unused last dimension for computation
  This makes it about 10% faster
* ErrorMinimizer: use weighted least squares optimization
  for this to be useful, an outlierfilter with continuous weights in [0,1]
  is needed.
* Merge branch 'syedharoonalam-syedharoonalam-patch-1'
* remove the typedef in icp_advance_api
* Update icp_advance_api.cpp
  Bug: 'CurrentBibliography' : ambiguous symbol. I simply changed typedef PointMatcherSupport::CurrentBibliography with different name (Currentbibliography) and its usage in same file. I think compiler is confused which symbol to refer.
* Update DataPoints.cpp
  Bug: Redefination of "getTimeViewByName". Changed line 648, return type to 'const'
* Update icpWithoutYaml.md
* Update icp_customized.cpp
* Create icpWithoutYaml.md
* Update index.md
* add example for icp without yaml file
* Update Datafilters.md
* fix bug when loading no descriptors in PCD files
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* add better support for VTK files saved by Paraview
* Update mkdocs.yml
* Update mkdocs.yml
* Update mkdocs.yml
* Merge pull request `#128 <https://github.com/samuelba/libpointmatcher/issues/128>`_ from 85pando/patch-1
  Fix: correct path to demo file
* Fix: also correct the label of the link
* Fix: correct path to demo file
* Update index.md
* removed bad use of title which confuse readthedoc
* Update LinkingProjects.md
* Update README.md
* Update index.md
* Update ApplicationsAndPub.md
* Update ApplicationsAndPub.md
* Update ApplicationsAndPub.md
* Update ApplicationsAndPub.md
* Update mkdocs.yml
* change location for mkdoc
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* Update mkdocs.yml
* Update README.md
* Rename Tutorials.md to index.md
* Create mkdocs.yml
* Merge pull request `#127 <https://github.com/samuelba/libpointmatcher/issues/127>`_ from ethz-asl/fix/WarningsCleanup
  Fixed warnings previously introduced
* fixed warnings previously introduced
* Update README.md
* add banner images
* Update LinkingProjects.md
* Update demo.pro
* Update README.md
* add new example code
* Merge pull request `#124 <https://github.com/samuelba/libpointmatcher/issues/124>`_ from ethz-asl/fix/pkg_config_shortcomings
  Some improvements for the pkg-config output
* Some improvements for the pkg-config output
* Merge pull request `#123 <https://github.com/samuelba/libpointmatcher/issues/123>`_ from ethz-asl/fix/revertAccidentalInstallCommand
  Removed outdated install line that was apparently accidentelly introduced in PR `#109 <https://github.com/samuelba/libpointmatcher/issues/109>`_ (d199aaaf9).
* Removed outdated install line that was apparently accidentally introduced in PR `#109 <https://github.com/samuelba/libpointmatcher/issues/109>`_ (d199aaaf9).
* Merge pull request `#122 <https://github.com/samuelba/libpointmatcher/issues/122>`_ from ethz-asl/feature/ElipsoidsFilter
  Feature/elipsoids filter
* addressed Renaud's comments
* Merge pull request `#109 <https://github.com/samuelba/libpointmatcher/issues/109>`_ from goldhoorn/master
  Added pkg-config file
* Added pkg-config file
  The pkg-config file helps external libraries to find and handle this library.
* adressed comments
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* Update ICPIntro.md
* fix mistake in module description
* some cosmetic changes and removing of feature fragments that will be adressed in subsequent development
* fixed some errors from merging
* merged master into feature branch
* Merge pull request `#120 <https://github.com/samuelba/libpointmatcher/issues/120>`_ from bkueng/fix_compilation
  fix compilation of utest, missing pthread library when linking
* Update ErrorMinimizersImpl.cpp
* Merge pull request `#121 <https://github.com/samuelba/libpointmatcher/issues/121>`_ from bkueng/fix_pointmatcher
  fix PointMatcher: SVD can return a reflection instead of a rotation
* fix compilation of utest, missing pthread library when linking
* fix PointMatcher: SVD can return a reflection instead of a rotation
  see also: http://igl.ethz.ch/projects/ARAP/svd_rot.pdf
* Merge pull request `#117 <https://github.com/samuelba/libpointmatcher/issues/117>`_ from oleg-alexandrov/point-to-plane-fix2
  Handle the degenerate situation when the residuals are 0.
* add missing documentation which was generating warnings
* add missing contructors for time and their unit tests
* ErrorMinimizersImpl.cpp: Handle the degenerate situation when the residuals are 0
  ErrorMinimizersImpl.cpp: Handle the degenerate situation when the residuals are 0
  Fix the degenerate situation of the identity transform
  Move the fix higher up
  Move the fix higher up
  Minimum version of libnabo is 1.0.6
  Rework the identity transform fix and add a unit test
  rm whitespace
* Merge branch 'time_support'
* Merge branch 'master' into time_support
* Merge pull request `#114 <https://github.com/samuelba/libpointmatcher/issues/114>`_ from oleg-alexandrov/fix0.99999
  No need to restrict sampling to 0.9999999
* No need to restrict sampling to 0.9999999
* Update README.md
* Merge pull request `#108 <https://github.com/samuelba/libpointmatcher/issues/108>`_ from ethz-asl/cleanup/removedUnusedTypedefs
  removed unused typedefs to get rid of compiler warnings
* resolved issue `#104 <https://github.com/samuelba/libpointmatcher/issues/104>`_ by removing MSVC specific block from CMakeList
* Fixed issue `#105 <https://github.com/samuelba/libpointmatcher/issues/105>`_ following kwill patch
* fixed issue `#105 <https://github.com/samuelba/libpointmatcher/issues/105>`_ by explicitly assigning the parameter map
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* used a better line to report ubuntu version
* Update Compilation.md
* change bash file to output a markdown table directly
* put back tests using isApprox()
* explicit row as unsigned to avoid compilation optimization
* Added a bash script to list system verions to simplify future bug report
* port to Eigen3 and removed EIGEN2_SUPPORT
* explicit rows and cols id has being unsigned to avoid weird optimization
* Merge pull request `#110 <https://github.com/samuelba/libpointmatcher/issues/110>`_ from ethz-asl/cleanup/removedDeadCmakeCode
  Removed dead cmake code
* Merge pull request `#107 <https://github.com/samuelba/libpointmatcher/issues/107>`_ from ethz-asl/fix/someCmakeFileInstallationProblems
  Fix/some cmake file installation problems
* Removed dead cmake code
* removed unused typedef to get rid of compiler warnings
* removed ${yaml-cpp-pm_LIB} from EXTERNAL_LIBS for installation in case of a shared library build.
  The installation will depend on the build files in case of a static library when using the contribute yaml library, because we don't install it.
  Maybe we should fix that too? But I don't think this is worth the effort, because a static installation is probably pretty unusual and only used by experts who can easily recover from that problem.
* also install libpointmatcherConfigVersion.cmake into share/${PROJECT_NAME}/cmake/
* * fixed which version of libpointmatcherConfig.cmake gets installed into share/${PROJECT_NAME}/cmake/ (it should be the install version (not depending on local names)
* white space cleanup
* fix issues with align_sequence example
* Merge pull request `#103 <https://github.com/samuelba/libpointmatcher/issues/103>`_ from carlosmccosta/master
  Added RandomSampling filter from PCL. Fixed loading ply files with color information. Improved Histogram stats.
* Fixed saving of PLY files. Updated PLY unit test. Removed dependency on
  time.h and stdlib.h
* Improved MaxPointCountDataPointsFilter to provide the expected
  funcionality.
  The previous implementation didn't achieved consistent results (didnt
  selected the specified number of points) given that it was reusing the
  RandomSamplingDataPointsFilter with a given probability instead of
  actually selecting the number specified points.
* Fixed loading of ply files with color information.
  - Added output of histogram stats to file.
  - Added a class member to ICPChainBase to allow access to the filtered
  reading DataPoints point cloud.
* Update Compilation.md
* Update CompilationWindows.md
* Update CompilationWindows.md
* updated windows compilation documentation
* Merge pull request `#102 <https://github.com/samuelba/libpointmatcher/issues/102>`_ from kwill/windows-doc-tweak
  Make Windows compilation instructions a bit more explicit
* make windows compilation instructions a bit more explicit
* merged
* fixed gestalt implementation
* Update ApplicationsAndPub.md
* squared the distance parameter for MaxDistOutlierFilter and MinDistOutlierFilter
* Update README.md
* Merge pull request `#95 <https://github.com/samuelba/libpointmatcher/issues/95>`_ from smichaud/master
  added cmake variable for the library (better flexibility)
* added cmake variable for the library (better flexibility)
* Merge branch 'master' of https://github.com/ethz-asl/libpointmatcher
* gave the possibility to extract residual from outside icp
* Merge pull request `#90 <https://github.com/samuelba/libpointmatcher/issues/90>`_ from wolfv/patch-1
  Small fix to docs
* Changed registrar.cpp to Registry.cpp
* Merge branch 'fix/invasiveYAMLNamespaceRedirection' into feature/ElipsoidsFilter
* Merge branch 'fix/invasiveYAMLNamespaceRedirection' into feature/ElipsoidsFilter
* Merge branch 'master' into feature/ElipsoidsFilter
* fixed random point selection in voxelGridFilter
* added positional means and knn for gestaltDataPointFilter
* moved helper functions to gestalt filter
* replaced placeholder points with transformed
* deleted some copied values from filter template
* first implementation of Gestalt descriptor (not tested).
* Merge branch 'master' into feature/ElipsoidsFilter
  Conflicts:
  examples/icp.cpp
  pointmatcher/DataPointsFiltersImpl.cpp
* added project files into gitignore list
* added storing of number of indices in knn (not operational yet) and 3rd method for downsamplonig a pointcloud -> selection of a random point within cell
* add a todo
* Merge /home/frank/research/nifti_svn/bleeding-edge/stacks/ethzasl_icp_mapping/libpointmatcher/upstream_src into time_support
* corrected wrong index for planarity in Ellipsoids filter
* planarity criterium implemented
* added minimal planarity for elipsoids
* refactor of covariance
* complet unit test for loading CSV
* add functionnality to load time in csv files
* added filling of weights descriptor
* change time type to signed 64 to allow duration
* resolve merge conflict
* added SurfelsFilter
* implemented new constructor for DataPoints taking time as arg
* really resolve conflict
* resolve conflict
* finish adding time to DataPoints
* generalize assertConsistency
* add time functionnalities in the header
* add time functionnalities in the header
* Contributors: 85pando, Andres Stepaniuk, Beat Küng, Ben Ballard, Capputchino, Carlos Costa, David Landry, David Seaward, Ellon Mendes, Francois Pomerleau, François Pomerleau, Hannes Sommer, HannesSommer, Jeremie Deray, Lorenz Wellhausen, MathLabu, Mathieu Labussiere, Matthias Goldhoorn, Oleg Alexandrov, Renaud Dube, Samuel Bachmann, Santiago Castro, Thomas Fischer, Tushar Dadlani, Wolf Vollprecht, gawela, lorenwel, pomerlef, smichaud, sven, syedharoonalam, vanurag

1.2.3 (2015-05-15)
------------------
* Support including other versions of YAML in compilation units that also include the YAML version packed with libpointmatcher (PR #80)
* Changed immutability concept for SupportLabel to support MSVC 2012 (#78)
* Fixed build system related bugs (#79, #70, ..).
* updated build_map example, added better error message, added better information prints
* cleaned CMakeList and added missing dependencies for external projetcs
* avoid possibility of building dynamic library on MacOS
* updated Mac build instructions
* Tim3xx laser support on Simple Noise filter (#64)
* Modified default covariance return in PointToPlaneWithCovErrorMinimizer (#59)
* update usage text and retab
* Removed compilation warnings
* add unit test for ICPSequence
* added application of reference data points filters for ICPSequence objects (#56)
* Merge branch 'master' of github.com:ethz-asl/libpointmatcher
* fix problem with libnabo linking (#54)
* Adapted the code to handle 2D point clouds and decided to split the initial/icp/complete transformation matrices in 3 different files. It should be easier to post process the transformations.
* Changed matrix for matrices as output suffix
* Changed the ICP example (pmicp) to accept initial translation/rotation input and allow to output the transformation matrices
* CutBelowLevelDataPointsFilter (PR #48)
* split unit tests (PR #47)
* Delete roadmap.txt
* change year to 2014
* correct bug in DataPoints operator==
* add a method to remove features or descriptors
* add empty function for removing features and descriptors
* add functions to DataPoints avoiding error on rows and cols
* fill missing documentation
* resolve warning from unsigned to int in IO.cpp
* add extra empty line in utest
* add extra unit tests and resolve remaining bugs
* Refactored how to load PLY files
* Allow 2D descriptors (##45)
* Allow saving 2D descriptors coming from a 2Dmap, that are converted to 3D when writing to the file but needed after if we want to load the map as 2D.
* Contributors: Francis Colas, Francisco J Perez Grau, François Pomerleau, HannesSommer, Philipp Kruesi, Renaud Dube, Simon Lynen, chipironcin, pomerlef, smichaud, v01d

1.2.2 (2014-08-05)
------------------
* Yaml-cpp0.3 now built with libpointmatcher for compatibility with newer Ubuntu systems using yaml-cpp0.5

1.2.1
-----------
* Fixed bug with soft outlier weights in error minimization
* Fixed some issues for releasing into ROS ecosystem
* Contributors: François Pomerleau, Mike Bosse, Samuel Charreyron, Simon Lynen
