# This file is only used by Platform.io
Import("env")

# General options that are passed to the C++ compiler
env.Append(CXXFLAGS=["-Wno-volatile"])
