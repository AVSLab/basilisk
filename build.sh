#!/bin/bash

cmake build/
xcodebuild -project build/AVS\ basilisk.xcodeproj -target ALL_BUILD
