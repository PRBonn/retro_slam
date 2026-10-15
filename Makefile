.PHONY: install uninstall editable clean build check

install:
	@pip3 install ./python/

uninstall:
	@pip3 -v uninstall retro_slam

editable:
	@pip3 install scikit-build-core pyproject_metadata pathspec pybind11 ninja cmake
	@pip3 install --no-build-isolation -e ./python/

build:
	@cmake -Bcpp/build -G Ninja cpp/ \
		-DCMAKE_BUILD_TYPE=Release \
		-DUSE_SYSTEM_EIGEN3=ON \
		-DUSE_SYSTEM_SOPHUS=ON \
		-DUSE_SYSTEM_G2O=ON \
		&& cmake --build cpp/build -j$(nproc)

clean:
	@rm -rf python/build cpp/build

check: editable
	@echo "Installation complete. Run 'retro_slam_pipeline --help' to see available commands."


# Docker commands
# Recursively fetch all files that requiere a new build, like C++ files, dockers, and build systems
CPP_SOURCES := $(shell find . -type f \( -name '*.cpp' -o -name '*.hpp' -o -name '*.cc' -o -name '*.h' \) -not -path '*/\.*')
YAML_SOURCES := $(shell find . -type f \( -name '*.yaml' -o -name '*.yml' \) -not -path '*/\.*')
CMAKELISTS := $(shell find . -name 'CMakeLists.txt' -not -path '*/\.*')
SOURCES := $(CPP_SOURCES) $(PY_SOURCES) $(YAML_SOURCES) $(CMAKELISTS)

DOCKERFILES := $(shell find . -name 'Dockerfile*' -not -path '*/\.*')

docker_exec=@docker compose exec retro_slam /ros_entrypoint.sh

.build_docker: $(SOURCES)
	@docker compose down
	@docker compose build --pull
	@docker compose run --rm retro_slam colcon build --packages-select retro_slam_ros --event-handlers console_direct+
	@touch $@

build_docker: .build_docker

start: build_docker
	@docker compose up retro_slam --detach --no-build --pull never

run: start
	$(docker_exec) bash -l

launch: start
	$(docker_exec) tmuxinator

stop:
	@docker compose down

docker_clean:
	@rm -rf .init .build install/ devel/ log/
