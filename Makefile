SHELL := /bin/bash

VERSION ?= dev
IMAGE ?= smartcar-ros2
VEHICLE ?= yunle@192.168.100.10
VEHICLE_DIR ?= /opt/smartcar
BAG ?= latest

COMPOSE_DEV := docker compose -f deploy/compose.dev.yaml

.PHONY: help build-dev build-runtime dev up down rviz rviz-stop record replay release deploy rollback health test clean tree

help:
	@echo "SmartCar targets:"
	@echo "  make build-dev                 Build the dev image"
	@echo "  make build-runtime VERSION=... Build a runtime image"
	@echo "  make dev                       Open the interactive dev container"
	@echo "  make up / down                 Start/stop dev sensors+slam containers"
	@echo "  make rviz / rviz-stop          Start/stop RViz container"
	@echo "  make record                    Record a bag on the dev machine"
	@echo "  make replay BAG=...            Replay a bag (one-off)"
	@echo "  make release VERSION=...       Build image + release bundle"
	@echo "  make deploy VERSION=...        rsync + load + compose up on vehicle"
	@echo "  make health / rollback         Run vehicle healthcheck / rollback"
	@echo "  make test                      colcon test in the dev container"
	@echo "  make clean / tree              Clean build artifacts / list files"

build-dev:
	docker build --target dev -t $(IMAGE):dev .

build-runtime:
	docker build --target runtime -t $(IMAGE):$(VERSION) .

dev:
	$(COMPOSE_DEV) run --rm dev

up:
	$(COMPOSE_DEV) --profile split up -d sensors slam

down:
	$(COMPOSE_DEV) --profile split down

rviz:
	xhost +si:localuser:root >/dev/null 2>&1 || true
	$(COMPOSE_DEV) --profile viz up -d rviz

rviz-stop:
	$(COMPOSE_DEV) --profile viz down
	xhost -si:localuser:root >/dev/null 2>&1 || true

record:
	$(COMPOSE_DEV) --profile record run --rm record

replay:
	BAG=$(BAG) $(COMPOSE_DEV) --profile replay run --rm replay

release:
	tools/release.sh $(VERSION)

deploy:
	deploy/deploy.sh $(VERSION)

rollback:
	ssh $(VEHICLE) "cd $(VEHICLE_DIR)/current && ./rollback.sh"

health:
	ssh $(VEHICLE) "cd $(VEHICLE_DIR)/current && ./healthcheck.sh"

test:
	$(COMPOSE_DEV) run --rm dev bash -lc 'source /opt/ros/$${ROS_DISTRO:-humble}/setup.bash && colcon test --merge-install && colcon test-result --verbose'

clean:
	rm -rf build install log

tree:
	find . -maxdepth 3 -type f | sort
