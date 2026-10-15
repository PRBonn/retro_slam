# MIT License
#
# # Copyright (c) 2026 Saurabh Gupta, University of Bonn
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
set(WITH_ITT OFF CACHE BOOL "Disable OpenCV ITT support" FORCE)
set(USE_SYSTEM_EIGEN3 ON CACHE BOOL "use system eigen3")
set(USE_SYSTEM_OPENCV ON CACHE BOOL "use system opencv")

include(FetchContent)
FetchContent_Declare(map_closures GIT_REPOSITORY https://github.com/PRBonn/MapClosures.git GIT_TAG v2.2.0 SOURCE_SUBDIR
                                                                                                          cpp)
FetchContent_MakeAvailable(map_closures)
