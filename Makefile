# Fastbot ROS2 Docker Makefile
.PHONY: build-gazebo build-slam build-web build-sim build-real build-real-slam build-all \
        up down ps shell-gazebo shell-slam shell-web push-sim push-real push-all

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

# ============ Build All ============
build-all: build-sim build-real build-real-slam

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

push-all: push-sim push-real
