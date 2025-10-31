variable "REGISTRY"    { default = "ghcr.io" }
variable "ORG"         { default = "dit-robotics" }
variable "VERSION"     { default = "latest" }
variable "BASE_IMAGE"  { default = "ros:humble" }
variable "USER_UID"    { default = "1000" }
variable "USER_GID"    { default = "1000" }

variable "LIBREALSENSE_VERSION"  { default = "2.55.1" }
variable "REALSENSE_ROS_VERSION" { default = "4.55.1" }

group "default" {
  targets = [ "librealsense", "realsense", "aruco"]
}

target "common" {
  context     = "."
  platforms   = ["linux/amd64", "linux/arm64"]
  cache-from  = ["type=gha"]
  cache-to    = ["type=gha,mode=max"]
  args = {
    BASE_IMAGE = "${BASE_IMAGE}"
    USER_UID   = "${USER_UID}"
    USER_GID   = "${USER_GID}"
  }
}

target "librealsense" {
  inherits   = ["common"]
  dockerfile = "docker/dockerfiles/Dockerfile.librealsense"
  tags       = ["${REGISTRY}/${ORG}/librealsense:${LIBREALSENSE_VERSION}"]
  args       = {
    LIBREALSENSE_VERSION  = "${LIBREALSENSE_VERSION}"
  }
}

target "realsense" {
  inherits   = ["common"]
  dockerfile = "docker/dockerfiles/Dockerfile.ros"
  target     = "realsense"
  tags       = ["${REGISTRY}/${ORG}/realsense:${VERSION}"]
  args = {
    USER = "realsense"
    LIBREALSENSE_VERSION  = "${LIBREALSENSE_VERSION}"
    REALSENSE_ROS_VERSION = "${REALSENSE_ROS_VERSION}"
  }
}

target "aruco" {
  inherits   = ["common"]
  dockerfile = "docker/dockerfiles/Dockerfile.ros"
  target     = "aruco"
  tags       = ["${REGISTRY}/${ORG}/aruco:${VERSION}"]
  args = {
    USER = "aruco"
  }
}
