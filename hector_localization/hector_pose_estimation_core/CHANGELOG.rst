^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_pose_estimation_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2021-02-16)
------------------
* Update maintainer email address
* Increase minimum CMake version to 3.0.2 to avoid the CMP0048 warning
  See
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
  for details.
* hector_pose_estimation_core: fix build error in Ubuntu Bionic
  The result of the expression is an Eigen type that cannot be converted to double directly in
  Eigen 3.3.4:
  /opt/hector/melodic/src/hector_localization/hector_pose_estimation_core/src/measurements/poseupdate.cpp: In instantiation of ‘double hector_pose_estimation::PoseUpdate::updateInternal(hector_pose_estimation::State&, const NoiseVariance&, const MeasurementVector&, const MeasurementMatrix&, const string&, double, hector_pose_estimation::PoseUpdate::JumpFunction) [with MeasurementVector = Eigen::CwiseBinaryOp<Eigen::internal::scalar_difference_op<double, double>, const Eigen::Matrix<double, 1, 1, 0, 1, 1>, const Eigen::Matrix<double, 1, 1, 0, 1, 1> >; MeasurementMatrix = Eigen::Matrix<double, 1, -1, 1, 1, 18>; NoiseVariance = Eigen::Matrix<double, 1, 1, 0, 1, 1>; std::__cxx11::string = std::__cxx11::basic_string<char>; hector_pose_estimation::PoseUpdate::JumpFunction = boost::function<void(hector_pose_estimation::State&, const Eigen::Matrix<double, -1, 1, 0, 19, 1>&)>]’:
  /opt/hector/melodic/src/hector_localization/hector_pose_estimation_core/src/measurements/poseupdate.cpp:201:181:   required from here
  /opt/hector/melodic/src/hector_localization/hector_pose_estimation_core/src/measurements/poseupdate.cpp:385:12: error: cannot convert ‘Eigen::internal::enable_if<true, const Eigen::CwiseBinaryOp<Eigen::internal::scalar_product_op<double, double>, const Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<double>, const Eigen::Matrix<double, 1, 1, 0, 1, 1> >, const Eigen::CwiseBinaryOp<Eigen::internal::scalar_difference_op<double, double>, const Eigen::Matrix<double, 1, 1, 0, 1, 1>, const Eigen::Matrix<double, 1, 1, 0, 1, 1> > > >::type {aka const Eigen::CwiseBinaryOp<Eigen::internal::scalar_product_op<double, double>, const Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<double>, const Eigen::Matrix<double, 1, 1, 0, 1, 1> >, const Eigen::CwiseBinaryOp<Eigen::internal::scalar_difference_op<double, double>, const Eigen::Matrix<double, 1, 1, 0, 1, 1>, const Eigen::Matrix<double, 1, 1, 0, 1, 1> > >}’ to ‘double’ in initialization
  double error2 = error.transpose() * Ix * (Ix + Iy).inverse() * Iy * error;
  ^~~~~~
  /opt/hector/melodic/src/hector_localization/hector_pose_estimation_core/src/measurements/poseupdate.cpp: In instantiation of ‘double hector_pose_estimation::PoseUpdate::updateInternal(hector_pose_estimation::State&, const NoiseVariance&, const MeasurementVector&, const MeasurementMatrix&, const string&, double, hector_pose_estimation::PoseUpdate::JumpFunction) [with MeasurementVector = Eigen::Matrix<double, 1, 1, 0, 1, 1>; MeasurementMatrix = Eigen::Matrix<double, 1, -1, 1, 1, 18>; NoiseVariance = Eigen::Matrix<double, 1, 1, 0, 1, 1>; std::__cxx11::string = std::__cxx11::basic_string<char>; hector_pose_estimation::PoseUpdate::JumpFunction = boost::function<void(hector_pose_estimation::State&, const Eigen::Matrix<double, -1, 1, 0, 19, 1>&)>]’:
  /opt/hector/melodic/src/hector_localization/hector_pose_estimation_core/src/measurements/poseupdate.cpp:234:154:   required from here
  /opt/hector/melodic/src/hector_localization/hector_pose_estimation_core/src/measurements/poseupdate.cpp:385:12: error: cannot convert ‘Eigen::internal::enable_if<true, const Eigen::CwiseBinaryOp<Eigen::internal::scalar_product_op<double, double>, const Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<double>, const Eigen::Matrix<double, 1, 1, 0, 1, 1> >, const Eigen::Matrix<double, 1, 1, 0, 1, 1> > >::type {aka const Eigen::CwiseBinaryOp<Eigen::internal::scalar_product_op<double, double>, const Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<double>, const Eigen::Matrix<double, 1, 1, 0, 1, 1> >, const Eigen::Matrix<double, 1, 1, 0, 1, 1> >}’ to ‘double’ in initialization
* Merge pull request `#16 <https://github.com/tu-darmstadt-ros-pkg/hector_localization/issues/16>`_ from nolanholden/catkin
  remove implicit operator bool on boost::shared_ptr<>
* hector_pose_estimation_core: replace nullptr by pre-C++11 expression
  The `nullptr` literal was only introduced in C++11:
  https://en.cppreference.com/w/cpp/language/nullptr
  but hector_localization does not (yet) require a C++11-enabled compiler.
* remove implicit operator bool on boost::shared_ptr<>
  See compilation error here:
  https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor/issues/82
* hector_pose_estimation_core: initialize matrices in EKF predictor and corrector instances to zero
* hector_pose_estimation_core: fixed symmetric matrix assertion in method TimeContinuousSystemModel\_<>::getSystemNoise()
* hector_pose_estimation_core: reverted integer constants in SystemModel and MeasurementModel classes to enum to avoid linker problems
  This fixes a regression from 81b976da6a54197357c2fa083d9c4e20e9121019.
* Contributors: Johannes Meyer, Nolan Holden

0.3.0 (2016-06-27)
------------------
* hector_pose_estimation_core: cleanup of Eigen MatrixBase and QuaternionBase plugins
* hector_pose_estimation_core: refactored vector and matrix classes
  Instead of inheriting from Eigen::Matrix<> types, hector_pose_estimation now simply defines
  typedefs for all kind of use cases. The former SymmetricMatrix\_<> and SkewSymmetricMatrix classes
  have been replaced by Eigen::MatrixBase extension (https://eigen.tuxfamily.org/dox-devel/TopicCustomizingEigen.html)
  and by a free function that returns a skew-symmetric matrix.
  The refactoring was required for the release in Ubuntu Xenial and Eigen 3.3. The previous code was
  too fragile in case of Eigen upgrades and did not compile in Xenial due to changes in the Eigen::internal::traits<T>
  interface.
* Contributors: Johannes Meyer

0.2.2 (2016-06-24)
------------------

0.2.1 (2015-11-08)
------------------
* hector_pose_estimation_core: use FindEigen3.cmake provided by Eigen
* hector_pose_estimation_core: removed REQUIRED option for Eigen3 to enable fallback to Eigen
* Contributors: Johannes Meyer

0.2.0 (2015-02-24)
------------------
* removed test_depend on hector_timing in package.xml
  It causes rosdep errors if hector_timing is not installed.
* use Quaterniond::fromRotationVector() to predict orientation in poseupdate and added predict_pose parameter to enable/disable pose prediction
* added fromRotationVector() and toRotationVector() member functions as a plugin to Eigen::QuaternionBase
* added QUIET flag when finding optional hector_timing
* removed inputs parameter from Filter and SystemModel member functions
  This was introduced in 4efd5fb98337650c7989944a9c611d6c2e86540e.
  System models with inputs have to get/register them in init(). There is no need to pass the Inputs collection through
  the whole call pipeline. Additionally, most overloads of SystemModel::get* methods have been removed. Implementations
  have to overwrite exactly one set of methods. Default implementations which return zero matrices have been moved to
  system_model.inl. All init parameters have now true as a default argument.
* added instrumentation for EIGEN_RUNTIME_NO_MALLOC checks
  The cached measurement noise matrix in class Measurement_<Model> does not have to be dynamically allocated.
  A test with EIGEN_RUNTIME_NO_MALLOC showed that hector_pose_estimation does not allocate heap memory after
  initialization and first update. Only smaller fixed-size vectors and matrices are constructed on the stack.
  See http://eigen.tuxfamily.org/index.php?title=FAQ#Where_in_my_program_are_temporary_objects_created.3F.
* some minor optimizations and cleanup
* added new publisher /geopose with type geographic_msgs/GeoPose
* instrumented code for timing measurements using hector_diagnostics
  See https://github.com/tu-darmstadt-ros-pkg/hector_diagnostics/tree/master/hector_timing.
* copy quaternion to the stack in State::updateOrientation()
  We observed segfaults due to wrong alignment of Eigen::Map<Eigen::Quaterniond>
  with Eigen 3.0.5 in precise.
* add ${CATKIN_DEVEL_PREFIX}/include as include directory explicitly
* fixed compile errors in precise and removed Index and Scalar using directives in matrix classes
* set initial yaw uncertainty to zero as we know this by definition
* fixed inversed bias correction in rate and acceleration output on topic /imu
* removed aligned new operators in matrix classes and replaced some typedefs by using directives
* added cmake cache variable for maximum number of state variables
  hector_pose_estimation only uses a maximum of 18 state variables. You should set this to a higher number to support bigger state variables,
  e.g. for plugins and/or application-specific extensions.
  The maximum state vector size is maximum state variables + 1 as the orientation is stored as a quaternion internally (four dimensions for three degrees of freedom).
* conditionally added some template specializations added in Eigen 3.1 to matrix.h to be compatible with earlier versions
* added world to nav transform broadcaster and global/reference (geographic_msgs/GeoPose) publisher
* switched sign of bias estimates and increase default accelerometer drift
  The inverted sign is consistent with the simulation environment.
  A positive bias means that the measured value is considered greater than the real value.
* reverted unfiltered state updates introduced in dynamic_state branch and properly reset system status
* do not integrate z rate during alignment or without yaw measurement
  This is consistent with the velocity and position updates.
* fixed magnetic heading calculation
* decreased default stddev of pseudo gravity update and enabled bias update for this measurement
* disabled velocity and position updates during alignment and fixed jacobian if velocity updates are disabled
* use OrientationPositionVelocityState as default state type
* prepend updated states in system status output
* properly reset internal predictor and corrector data
* fixed orientation noise calculation
* added state type OrientationPositionVelocityState (without rate)
* use preallocated Eigen types
  The maximum number of state variables in 24.
* Added missing orientation-orientation derivative in GenericQuaternionSystemModel
* Readded dimension paramters to SystemModel templates
* Readded class BaseState and State::base() accessors (currently unused)
* Added typed getters PoseEstimation::getSystem_<T>(name) and PoseEstimation::getMeasurement_<T>(name)
* IMU-based prediction is now in the GenericQuaternionSystemModel class again.
  The accelerometer and gyro models are added automatically if GenericQuaternionSystemModel::init() found an IMU input.
  GenericQuaternionSystemModel calls methods on the IMU model classes to get the errors, derivatives and noise.
  The IMU models themselves do not add
* Added class SkewSymmetricMatrix
* Avoid calling SymmetricMatrix::symmetric() as, surprisingly, this method seems to be quite expensive. Symmetry is only
  enforced if assigned from a matrix which is not a SymmetricMatrix.
* Added Eigen optimizations (avoid copies and added alignment macros)
* Moved system and measurement status management from class Filter to PoseEstimation
* added parameter 'use_bias' to measurement models that can incorporate biases
  * Only the AccelerometerModel and GyroModel add new substates named `name + "_bias"`. There could be multiple accelerometer and gyro models.
  * The default name for these two models is "accelerometer" and "gyro" and the default bias substates are called "accelerometer_bias" and "gyro_bias".
  * Measurement models Gravity, Zerorate and Rate have a new string parameter 'use_bias' which defaults to "accelerometer_bias" or "gyro_bias", respectively.
  * Only if this substate is found the biases are considered in the measurement models. Otherwise the models will not use a bias.
  The gravity model uses the bias only to calculate the expected measurement vector but does not populate the respective entries in the measurement matrix.
  * The initialization order of systems and measurements in PoseEstimation::init() has been swapped. First systems to register new states, then measurements.
  * The 'imu' input is not added automatically anymore by the GenericQuaternionSystemModel, but is only used if it was already registered before initialization.
  The whole patch aims at incresing the flexibility, e.g. to use the GenericQuaternionSystemModel without an IMU and/or with other force and torque inputs.
* moved check if a pseudo update is required from Model::active() to PoseEstimation::update()
* do not scale variance of magneto measurement in normalized mode
  With this patch the magnetic/stddev parameter is considered as the standard deviation of the
  normalized measurement vector and not of the raw measurement if the magnitude of the field
  is not modeled (the magnetic/magnitude parameter is 0). If the magnitude is set, the normalization
  is inactive and the stddev relates to the original measurement vector.
* fixed missing return value in EKF::predict()
* fixed calculation of reference magnetic field vector pointing to magnetic north and inversed transformation to body frame
* reintroduced traits for SystemModel and MeasurementModel implementations
* properly cleanup the cached noise variance in Measurement_<Model>::reset() default implementation
* removed the SymmetricMatrix(dim,value) constructor
* added a method to get the residual of the last measurement update to Filter::Corrector_<Model>
* do not return reference to (potential) temporary in Collection::add()
  Collection::add() returned a const reference to its argument, which could be a temporary depending on how it is called.
* fixed rotation in z velocity update in GroundVehicleModel
* use pointers in boost::bind() to avoid copies of models
* first functional version with multiplicative EKF
* differentiate between vector and covariance dimension in State, SubState, SystemModel and MeasurementModel (work in progress)
* use average values to symmetrize matrices and disabled symmetry assertion
* fixed DenseBase compiler errors with Eigen 3.2.1 and clang (fix #4)
* added support for dynamic states (full, orientation only, position/velocity only, ...)
* fixed compilation error in Ubuntu Quantual with libeigen3-dev version 3.1.0~beta1-1ubuntu1
* Contributors: Johannes Meyer

0.1.5 (2014-10-02)
------------------
* fixed rate conversion to nav frame for the state message
* initialize reference values to NaN instead of 0.0 and added measurement/auto_* parameters consitently
  Added parameters:
  - gps/auto_reference
  - height/auto_elevation
  - baro/auto_elevation
  Already existed before:
  - magnetic/auto_heading
  All auto_* parameters are true by default.
* Contributors: Johannes Meyer

0.1.4 (2014-08-28)
------------------
* calculate euler angles directly in pose update without Eigen
  Eigen's eulerAngles() returns wrong yaw angles in Trusty for some reason.
* Contributors: Johannes Meyer

0.1.3 (2014-07-09)
------------------

0.1.2 (2014-06-02)
------------------
* added cmake_modules dependency for the Eigen cmake config
* Contributors: Johannes Meyer

0.1.1 (2014-03-30)
------------------
* Fixed boost 1.53 issues
  changed boost::shared_dynamic_cast to boost::dynamic_pointer_cast and
  boost::shared_static_cast to boost::static_pointer_cast
* hector_pose_estimation_core: rotate rate vector to nav frame in PoseEstimation::getState()
  All vectors in state messages (e.g. on topic /state) are given in nav frame. The rate vector
  has not been converted from body until now.
* Contributors: Christopher Hrabia, Johannes Meyer

0.1.0 (2013-09-03)
------------------
* catkinized stack hector_localization
