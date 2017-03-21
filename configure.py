#!/usr/bin/python

from ninja_syntax import Writer
import os, sys

source_dirs = [
        ".",
        "./VectorLib",
        "driverlib",
        "inc",
        "drivers",
        "drivers/SPI",
        "utils"
        ]

include_dirs = source_dirs

libraries = [
        ]

defines = [
        ]

def subst_ext(fname, ext):
    return os.path.splitext(fname)[0] + ext

def get_sources():
    fnames = []
    for d in source_dirs:
        for f in os.listdir(d):
            fnames.append(os.path.join(d, f))
    return fnames

def get_includes():
    return " ".join(map(lambda x : "-I"+x, source_dirs + include_dirs))

def get_libs():
    return " ".join(libraries)

def get_defines():
    return " ".join(map(lambda x : "-D"+x, defines))

with open("build.ninja", "w") as buildfile:
    n = Writer(buildfile)

    # Variable declarations
    n.variable("tc_prefix", "arm-none-eabi-")
    n.variable("cxxflags", "-g -Wall -O1 -std=c++14 -fno-rtti -fno-exceptions " +
                           "-ffunction-sections -fdata-sections -mthumb " +
                           "-mcpu=cortex-m4 -mfloat-abi=hard " +
                           "-mfpu=fpv4-sp-d16 -fsingle-precision-constant " +
                           "-DF_CPU=80000000L -DPART_TM4C123GH6PM -nostdlib " +
                           get_includes() + " " + get_defines())
    n.variable("cflags", "-g -Wall -O1 -std=c99 -fdata-sections -mthumb " +
                         "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 " +
                         "-fsingle-precision-constant -DF_CPU=80000000L " +
                         "-DPART_TM4C123GH6PM -nostdlib " + get_includes() +
                         " " +  get_defines())
    n.variable("lflags", "-g -Wl,--gc-sections --specs=nano.specs -T " +
                         "tm4c123gh6pm.ld -Wl,--entry=ResetISR -mthumb " +
                         "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 " +
                         "-fsingle-precision-constant")
    n.variable("lflags_libs", "-lm -lstdc++ -lc")
    n.variable("libs", get_libs())

    # Rule declarations
    n.rule("cxx",
           command = "${tc_prefix}g++ $cxxflags -c $in -o $out")

    n.rule("cc",
           command = "${tc_prefix}gcc $cflags -c $in -o $out")

    n.rule("cl",
           command = "${tc_prefix}gcc $lflags -o $out $in $libs $lflags_libs")

    n.rule("ocb",
           command = "${tc_prefix}objcopy -O binary $in $out")

    n.rule("cdb",
           command = "ninja -t compdb cc cxx > cc_preexp.json")

    n.rule("cdb_e",
           command = "cat cc_preexp.json | ./expand_compdb.py > " +
                     "compile_commands.json")

    n.rule("cscf",
            command = "find " + " ".join(set(source_dirs + include_dirs)) +
                      " -regex \".*\\(\\.c\\|\\.h\\|.cpp\\|.hpp\\)$$\" -and " +
                      "-not -type d > $out")

    n.rule("cscdb",
           command = "cscope -bq")

    # Build rules
    n.build("cc_preexp.json", "cdb")
    n.build("compile_commands.json", "cdb_e", "cc_preexp.json")
    n.build("cscope.files", "cscf")
    n.build(["cscope.in.out", "cscope.po.out", "cscope.out"], "cscdb",
            "cscope.files")

    objects = []

    def cc(name):
        ofile = subst_ext(name, ".o")
        n.build(ofile, "cc", name)
        objects.append(ofile)
    def cxx(name):
        ofile = subst_ext(name, ".o")
        n.build(ofile, "cxx", name)
        objects.append(ofile)
    def cl(oname, ofiles):
        n.build(oname, "cl", ofiles)

    sources = get_sources()
    map(cc, filter(lambda x : x.endswith(".c"), sources))
    map(cxx, filter(lambda x : x.endswith(".cpp"), sources))

    cl("main.elf", objects)

    n.build("main.bin", "ocb", "main.elf")

