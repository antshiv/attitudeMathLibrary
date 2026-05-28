BUILD_DIR ?= build
CMAKE ?= cmake
CTEST ?= ctest

.PHONY: all help configure build test test-list test-list-details test-quaternion-relative test_quaternion_relative run-quaternion-relative run_quaternion_relative clean distclean

all: build

help:
	@printf "Attitude Math Library build targets\n"
	@printf "\n"
	@printf "  make                         Configure and build the library/tests\n"
	@printf "  make build                   Configure and build the library/tests\n"
	@printf "  make test                    Run the full CTest suite\n"
	@printf "  make test-list               List discovered CTest tests\n"
	@printf "  make test-list-details       Explain what each current test covers\n"
	@printf "  make test-quaternion-relative Run only the current-to-target quaternion test\n"
	@printf "  make test_quaternion_relative Alias for test-quaternion-relative\n"
	@printf "  make run-quaternion-relative  Build and run ./$(BUILD_DIR)/test_quaternion_relative\n"
	@printf "  make run_quaternion_relative  Alias for run-quaternion-relative\n"
	@printf "  make clean                   Clean compiled objects inside $(BUILD_DIR)\n"
	@printf "  make distclean               Remove $(BUILD_DIR) completely\n"
	@printf "\n"
	@printf "Variables:\n"
	@printf "  BUILD_DIR=build_debug        Use a different build directory\n"
	@printf "  CMAKE=cmake                  Override cmake executable\n"

configure: $(BUILD_DIR)/Makefile

$(BUILD_DIR)/Makefile: CMakeLists.txt
	$(CMAKE) -S . -B $(BUILD_DIR)

build: configure
	$(CMAKE) --build $(BUILD_DIR)

test: build
	$(CTEST) --test-dir $(BUILD_DIR) --output-on-failure

test-list: configure
	$(CTEST) --test-dir $(BUILD_DIR) -N

test-list-details:
	@printf "Current test map\n"
	@printf "\n"
	@printf "  test_attitude                  Broad attitude conversion smoke tests\n"
	@printf "  test_attitude_degrees          Degree-based attitude conversion check\n"
	@printf "  test_dcm_orthogonal            DCM orthogonality validation\n"
	@printf "  test_euler                     Euler conversion tests\n"
	@printf "  test_euler_random              Randomized Euler conversion tests\n"
	@printf "  test_quaternion                Quaternion conversion/composition tests\n"
	@printf "  test_quaternion_inverse_axis   Quaternion inverse and axis-angle tests\n"
	@printf "  test_quaternion_relative       Current orientation -> target orientation correction\n"
	@printf "  test_quaternion_rotate         Optimized quaternion vector rotation\n"
	@printf "  test_quaternion_rotate_explicit_demo Explicit q*v*q conjugate debug demo\n"
	@printf "  test_quaternion_slerp          SLERP interpolation tests\n"
	@printf "  test_rotation                  Rotation helper tests\n"

test-quaternion-relative: build
	$(CTEST) --test-dir $(BUILD_DIR) -R '^test_quaternion_relative$$' --output-on-failure

test_quaternion_relative: test-quaternion-relative

run-quaternion-relative: build
	./$(BUILD_DIR)/test_quaternion_relative

run_quaternion_relative: run-quaternion-relative

clean: configure
	$(CMAKE) --build $(BUILD_DIR) --target clean

distclean:
	rm -rf $(BUILD_DIR)
