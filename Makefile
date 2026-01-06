# Makefile
.PHONY: build-gazebo build-slam build-web build-all up down ps

# Platform for consistent builds (amd64 = standard Linux x86_64)
PLATFORM := linux/amd64

# ============ Build Commands ============
build-gazebo:
	docker build --platform $(PLATFORM) -f ./docker/simulation/Dockerfile.gazebo -t legalaspro/fastbot:fastbot-ros2-gazebo .

build-slam:
	docker build --platform $(PLATFORM) -f ./docker/simulation/Dockerfile.slam -t legalaspro/fastbot:fastbot-ros2-slam .

build-web:
	docker build --platform $(PLATFORM) -f ./docker/simulation/Dockerfile.web -t legalaspro/fastbot:fastbot-ros2-webapp .

build-all: build-gazebo build-slam build-web

# ============ Run Commands ============
# Allow X11 access before running (for GUI)
x11-allow:
	xhost +local:docker

up: x11-allow
	docker compose -f ./docker/simulation/docker-compose.yaml up

down:
	docker compose -f ./docker/simulation/docker-compose.yaml down

ps:
	docker compose -f ./docker/simulation/docker-compose.yaml ps

# ============ Shell Access ============
shell-gazebo:
	docker exec -it fastbot-gazebo bash

shell-slam:
	docker exec -it fastbot-slam bash

shell-web:
	docker exec -it fastbot-web bash