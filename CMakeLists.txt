idf_component_register(SRCS "src/atapi_host.c"
                            "src/atapi_mmc.c"
                            "src/atapi_cdtext.c"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "private_include"
                       REQUIRES usb
                       )
