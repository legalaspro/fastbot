# Fastbot ROS2 Docker Makefile
.PHONY: build-gazebo build-slam build-web build-sim build-real build-real-slam build-remote build-all \
        up down ps shell-gazebo shell-slam shell-web shell-remote \
        push-sim push-real push-remote push-all

# ============ Configuration ============
PLATFORM_AMD64 := linux/amd64
PLATFORM_ARM64 := linux/arm64
REGISTRY       := legalaspro/fastbot
SIM_COMPOSE    := ./docker/simulation/docker-compose.yaml

# ============ Simulation Builds (amd64) ============
build-gazebo:
	docker build --platform $(PLATFORM_AMD64) \
		-f ./docker/simulation/Dockerfile.gazebo \
		-t $(REGISTRY):fastbot-ros2-gazebo .

build-slam:
	docker build --platform $(PLATFORM_AMD64) \
		-f ./docker/simulation/Dockerfile.slam \
		-t $(REGISTRY):fastbot-ros2-slam .

build-web:
	docker buildx build --platform $(PLATFORM_AMD64) \
		-f ./docker/simulation/Dockerfile.web \
		-t $(REGISTRY):fastbot-ros2-webapp --load .

build-sim: build-gazebo build-slam build-web

# ============ Real Robot Builds (arm64) ============
build-real:
	docker build --platform $(PLATFORM_ARM64) \
		-f ./docker/real/Dockerfile.robot \
		-t $(REGISTRY):fastbot-ros2-real .

build-real-slam:
	docker build --platform $(PLATFORM_ARM64) \
		-f ./docker/real/Dockerfile.slam \
		-t $(REGISTRY):fastbot-ros2-slam-real .

# ============ Remote Dev Build (multi-platform) ============
# Multi-platform requires --push (can't --load multiple platforms locally)
build-remote:
    docker buildx build --platform linux/amd64,linux/arm64 \
        --provenance=false --sbom=false \
        -f ./docker/real/Dockerfile.remote \
        -t $(REGISTRY):fastbot-ros2-remote --push .

# Build for local use only (native platform - for devcontainer)
build-remote-local:
	docker build \
		-f ./docker/real/Dockerfile.remote \
		-t $(REGISTRY):fastbot-ros2-remote .

# ============ Build All ============
build-all: build-sim build-real build-real-slam build-remote

# ============ Compose Commands ============
up:
	docker compose -f $(SIM_COMPOSE) up

down:
	docker compose -f $(SIM_COMPOSE) down

ps:
	docker compose -f $(SIM_COMPOSE) ps

# ============ Shell Access ============
shell-gazebo:
	docker exec -it fastbot-gazebo bash

shell-slam:
	docker exec -it fastbot-slam bash

shell-web:
	docker exec -it fastbot-web bash

# ============ Push Commands ============
push-sim:
	docker push $(REGISTRY):fastbot-ros2-gazebo
	docker push $(REGISTRY):fastbot-ros2-slam
	docker push $(REGISTRY):fastbot-ros2-webapp

push-real:
	docker push $(REGISTRY):fastbot-ros2-real
	docker push $(REGISTRY):fastbot-ros2-slam-real

push-remote:
	docker push $(REGISTRY):fastbot-ros2-remote

push-all: push-sim push-real push-remote

# ============ Remote Dev Commands ============
# Use VS Code devcontainer instead: .devcontainer/real/
# Open folder in VS Code -> "Dev Containers: Reopen in Container"
shell-remote:
	docker exec -it fastbot-real-devcontainer bash
