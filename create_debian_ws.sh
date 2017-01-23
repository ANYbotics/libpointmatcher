#!/usr/bin/env bash

PACKAGE="libpointmatcher"

declare -a DIRECTORIES=(
  "contrib"
  "debian"
  "examples"
  "evaluations"
  "doc"
  "pointmatcher"
  "utest"
)

declare -a FILES=(
  "CMakeLists.txt"
  "CHANGELOG.rst"
  "package.xml"
  "cmake_uninstall.cmake.in"
  "Doxyfile.in"
  "libpointmatcherConfig.cmake.in"
  "libpointmatcherConfigVersion.cmake.in"
  "mkdocs.yml"
  "README.md"
  "pointmatcher.pc.in"
  "UseDoxygen.cmake"
)

rm -rf ws

mkdir -p ws/${PACKAGE}

for DIRECTORY in "${DIRECTORIES[@]}"; do
    cp -R ${DIRECTORY} "ws/"${PACKAGE}
done

for FILE in "${FILES[@]}"; do
    cp ${FILE} "ws/"${PACKAGE}
done

cd ws/${PACKAGE}

tar -cJ --exclude='./debian' -f ../libpointmatcher_1.2.0.orig.tar.xz .

#pdebuild --architecture amd64 --buildresult ~/pbuilder_ws/ws/libpointmatcher/ws/ --pbuilderroot "sudo DIST=xenial ARCH=amd64"