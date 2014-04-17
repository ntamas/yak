find_package(PkgConfig)
pkg_check_modules(PC_EIGEN3 eigen3)
set(EIGEN3_DEFINITIONS ${PC_EIGEN3_CFLAGS_OTHER})

find_path(EIGEN3_INCLUDE_DIR Eigen/Dense
	HINTS ${PC_EIGEN3_INCLUDE_DIRS})

set(EIGEN3_LIBRARIES "")
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen3 DEFAULT_MSG EIGEN3_INCLUDE_DIR)

mark_as_advanced(EIGEN3_INCLUDE_DIR)
