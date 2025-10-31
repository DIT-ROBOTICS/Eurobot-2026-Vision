include .env
export

BAKE_FILE ?= docker/dockerfiles/docker-bake.hcl

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

.PHONY: help login builder build build-librealsense tags

help:
	@echo "Usage: make [target]"
	@echo "Makefile targets:"
	@echo "  make login   |  Login GHCR (CR_PAT token in CLI)"
	@echo "  make builder |  Make builder: librealsense"
	@echo "  make build   |  Make images: realsense, aruco"
	@echo "  make tags    |  Show image tags"
	@echo "  Variables:  PUSH=1 to registry"
	@echo "              LOAD=1 to local (single platform)"
	@echo "              PLATFORMS=... architecture"
	@echo "  Other Vars: VERSION / LIBREALSENSE_VERSION / REALSENSE_ROS_VERSION, etc."

login:
	@echo "Logging into $(REGISTRY)..."
	@if [ -n "$$CR_PAT" ]; then \
	  echo "$$CR_PAT" | docker login $(REGISTRY) -u $$USER --password-stdin ; \
	else \
	  docker login $(REGISTRY) ; \
	fi

builder:
	docker buildx bake $(BAKE_FLAGS) librealsense

build: build-librealsense
	docker buildx bake $(BAKE_FLAGS) realsense aruco

tags:
	@echo "librealsense: $(REGISTRY)/$(ORG)/librealsense:$(LIBREALSENSE_VERSION)"
	@echo "realsense:    $(REGISTRY)/$(ORG)/realsense:$(VERSION)"
	@echo "aruco:        $(REGISTRY)/$(ORG)/aruco:$(VERSION)"