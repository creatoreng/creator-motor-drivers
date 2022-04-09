#!/bin/bash

set -e

build_targets=${@-all}

mkdir -p build
pushd build >/dev/null
cmake ../
make -j4 ${build_targets}
popd >/dev/null

exit $?
