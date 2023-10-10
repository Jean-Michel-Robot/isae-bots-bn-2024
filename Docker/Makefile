# Makefile for ISAEBOTS development
#
# Usage:
# 	make <target> <arg>=<val>
# 	
#
# Intallation:
#	make build-core
# 	make build-base
#	make create-container
#	make main

# Docker intern variables
IMAGE_NAME = isaebots_desktop_env
CONTAINER_NAME = isaebots
PS_ID = null
CMD = bash
CORE_DOCKERFILE = ${PWD}/docker/dockerfile.core
BASE_DOCKERFILE = ${PWD}/docker/dockerfile.base

# Setup Docker volumes and env variables
DOCKER_VOLUMES = \
	--volume="${PWD}/dev":"/app/dev" \
	--volume="/dev":"/dev" \
	--volume="${PWD}/scripts":"/app/scripts" \
	--volume="/tmp/.X11-unix":"/tmp/.X11-unix"
#	--volume="/var/run/dbus/system_bus_socket":"/var/run/dbus/system_bus_socket"

DOCKER_ENV_VAR = \
	-e DISPLAY=${DISPLAY} \
	--env="WDIR=dev" 

# .PHONY means that the Makefile command doesn't use any file as a source
.PHONY: help
help:
	@echo "=== HELP message ===================================="
	@echo "make build-core    for Ubuntu-20.04 + ROS-noetic img "
	@echo "make build-base    for Desktop ISAEBOTS dev env img  "
	@echo ""
	@echo "make kill          to kill any running env container "
	@echo "make term          to init a terminal in env Desktop "
	@echo "====================================================="

#############################################################
# SETUP
#############################################################

test:
	@echo ${CORE_DOCKERFILE} ${IMAGE_NAME}

# Build the core image
.PHONY: build-core
build-core:
	@echo ${CORE_DOCKERFILE} ${IMAGE_NAME}
	@docker build -f ${CORE_DOCKERFILE} -t ${IMAGE_NAME}_core .

# Build the base image (depends on core image build)
.PHONY: build-base
build-base: build-core
	@docker build -f ${BASE_DOCKERFILE} -t ${IMAGE_NAME}_base .

.PHONY: build-image-desktop
build-image-desktop:
	@docker build -f ${PWD}/docker/dockerfile_desktop.full -t isaebots_desktop_env_full .

.PHONY: build-image-pi
build-image-pi:
	@docker buildx build --platform=linux/arm/v7 -f ${PWD}/docker/dockerfile_pi.full -t isaebots_pi_env_full . --load


.PHONY: create-container
create-container:

#	the 'privileged' flag is necessary ? otherwise we get a Dbus error, but the kernel is more exposed this way
#	we should log in as a user and not root (preferable)

#	Check if container has been created, if not create it
	@if [ -z $$(docker ps -aqf name=$(CONTAINER_NAME)) ]; then \
        echo "Creating container $(CONTAINER_NAME) ..."; \
		docker run -it --net=host \
		--name ${CONTAINER_NAME} \
		${DOCKER_VOLUMES} \
		${DOCKER_ENV_VAR} \
		-u 0 \
		${IMAGE_NAME}_base \
		"${CMD}"; \
		echo "Created container successfully"; \
    else \
        echo "Container $(CONTAINER_NAME) is already created"; \
    fi

#############################################################
# TASKS
#############################################################

# Kill any running Docker containers
#.PHONY: kill
# /!\ doesn't kill a running container, only stopped containers (to do it use docker kill $(docker container ls -q)
#kill: 
#	@echo "Closing already running container"
#	@docker container prune -f
	

# Removes the container before running it again to make a new one
.PHONY: clear-container
clear-container:
	@if [ -z $$(docker ps -aqf name=$(CONTAINER_NAME)) ]; then \
		echo "Container $(CONTAINER_NAME) doesn't exist yet"; \
	else \
		echo "Replacing container $(CONTAINER_NAME) with a new one ..."; \
		docker container rm $(CONTAINER_NAME) > /dev/null; \
		docker run -it --net=host \
		--name ${CONTAINER_NAME} \
		${DOCKER_VOLUMES} \
		${DOCKER_ENV_VAR} \
		-u 0 \
		${IMAGE_NAME}_base \
		"${CMD}"; \
	fi


# Start a terminal inside the Docker container, and then close the container (difference with make term)
.PHONY: main
main: create-container
#	Check if container is running
	@if [ -z $$(docker ps -qf name=$(CONTAINER_NAME)) ]; then \
        echo "Starting container $(CONTAINER_NAME) ..."; \
		docker start $(CONTAINER_NAME) > /dev/null; \
    else \
        echo "Container $(CONTAINER_NAME) is already running"; \
    fi

	@docker exec -it $(shell docker ps -aqf "name=${CONTAINER_NAME}") bash -c "source /opt/ros/noetic/setup.bash; ${CMD}"

	@echo "Stopping container $(CONTAINER_NAME) ..."
	@docker kill $(CONTAINER_NAME) > /dev/null;



# Start a terminal inside the Docker container, doesn't close it at on exit
.PHONY: term
term:
#	Check if container is running
	@if [ -z $$(docker ps -qf name=$(CONTAINER_NAME)) ]; then \
        echo "Container $(CONTAINER_NAME) is not started yet"; \
    else \
        docker exec -it $(shell docker ps -aqf "name=${CONTAINER_NAME}") bash -c "source /opt/ros/noetic/setup.bash; ${CMD}"; \
    fi

	
# Terminal used for the simulation with special bindkeys
.PHONY: sim_term
sim_term:
	@docker exec -it $(shell docker ps -aqf "name=${CONTAINER_NAME}") bash --rcfile ./dev/src/uix/log/simTerm_rc.sh


# Running container called NAME :
# docker ps -aqf status=running --filter name=NAME
# OR
# docker ps -qf name=NAME

# Stopped container called NAME
# docker ps -aqf status=exited --filter name=NAME
