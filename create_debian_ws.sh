#!/usr/bin/env bash

PACKAGE="libpointmatcher"

declare -a DIRECTORIES=(
  "contrib"
  "debian"
  "doc"
  "evaluations"
  "examples"
  "pointmatcher"
  "utest"
)

declare -a FILES=(
  "CHANGELOG.rst"
  "cmake_uninstall.cmake.in"
  "CMakeLists.txt"
  "Doxyfile.in"
  "libpointmatcherConfig.cmake.in"
  "libpointmatcherConfigVersion.cmake.in"
  "mkdocs.yml"
  "package.xml"
  "pointmatcher.pc.in"
  "README.md"
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

tar -cJ --exclude='./debian' -f ../libpointmatcher_1.2.4.orig.tar.xz .

#pdebuild --architecture amd64 --buildresult ~/pbuilder_ws/ws/libpointmatcher/ws/ --pbuilderroot "sudo DIST=xenial ARCH=amd64"