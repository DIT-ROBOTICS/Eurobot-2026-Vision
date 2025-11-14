include .env
export

BAKE_FILE ?= docker/dockerfiles/docker-bake.hcl

SERVICE := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))

# buildx flags
BAKE_FLAGS := -f $(BAKE_FILE) --progress=auto
BAKE_FLAGS += --set *.platform="$(PLATFORMS)"

ifeq ($(PUSH),1)
BAKE_FLAGS += --push
endif

ifeq ($(LOAD),1)
# Notice: --load supports only single platform builds (NO multi-platform)
BAKE_FLAGS += --load
endif

# compose flags
COMPOSE_DEV ?= docker/composes/compose.dev.yml
COMPOSE_RUN ?= docker/composes/compose.run.yml


.PHONY: help login builder build dev down tags

%:
	@:

help:
	@echo "Usage: make [target]"
	@echo "Makefile targets:"
	@echo "  make login        |  Login GHCR (CR_PAT token in CLI)"
	@echo "  make builder      |  Make builder: librealsense"
	@echo "  make build [opt]  |  Make images"
	@echo "  make dev   [opt]  |  Compose development up"
	@echo "  make down  [opt]  |  Compose down"
	@echo "  make tags         |  Show image tags"
	@echo "  [opt]: realsense, aruco"
	@echo "  Variables:  PUSH=1 to registry"
	@echo "              LOAD=1 to local (single platform)"
	@echo "              PLATFORMS=... architecture"
	@echo "  Other Vars: VERSION / LIBREALSENSE_VERSION / REALSENSE_ROS_VERSION, etc."

login:
	@echo "Logging into $(REGISTRY)..."
ifeq ($(CR_PAT),)
	docker login $(REGISTRY)
else
	@echo "$(CR_PAT)" | docker login $(REGISTRY) -u "$(USER)" --password-stdin
endif

builder:
	docker buildx bake $(BAKE_FLAGS) librealsense

build:
ifeq ($(SERVICE),)
	docker buildx bake $(BAKE_FLAGS)
else
	docker buildx bake $(BAKE_FLAGS) $(SERVICE)
endif

dev: 
ifeq ($(SERVICE),)
	@echo ">> Running ALL Services ..."
	docker compose -f $(COMPOSE_DEV) up -d
else
	@echo ">> Running Service: $(SERVICE)"
	docker compose -f $(COMPOSE_DEV) up $(SERVICE) -d
endif

down:
ifeq ($(SERVICE),)
	@echo ">> Stopping ALL Services ..."
	docker compose -f $(COMPOSE_DEV) down
else
	@echo ">> Stopping Service: $(SERVICE)"
	docker compose -f $(COMPOSE_DEV) stop $(SERVICE)
	docker compose -f $(COMPOSE_DEV) rm -f $(SERVICE)
endif
	@echo ">> Cleaning tmpfs volumes ..."
	@vols=$$(docker volume ls -qf dangling=true | grep -v "_install" || true); \
	docker volume rm $$vols

tags:
	@echo "librealsense: $(REGISTRY)/$(ORG)/librealsense:$(LIBREALSENSE_VERSION)"
	@echo "realsense:    $(REGISTRY)/$(ORG)/realsense:$(VERSION)"
	@echo "aruco:        $(REGISTRY)/$(ORG)/aruco:$(VERSION)"