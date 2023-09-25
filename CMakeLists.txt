set(srcs
    "src/as5600.c")

set(include_dirs "include")

set(requires "driver")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES "${requires}")