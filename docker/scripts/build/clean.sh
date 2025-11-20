#!/usr/bin/env bash
set -e

apt-get clean -y 
rm -rf /var/lib/apt/lists/* \
       /var/cache/apt/* \
       /var/cache/debconf/* \
       /var/log/apt/* \
       /var/log/dpkg.log 