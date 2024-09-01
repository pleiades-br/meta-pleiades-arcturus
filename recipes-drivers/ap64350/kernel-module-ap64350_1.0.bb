#
# EG91 Chip Initialization driver
#

# Bitbake class(es)
inherit module 

# Dependencies
DEPENDS = "virtual/kernel"

# Metadata
SUMMARY = "Ap64350 enable v5 driver"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

SRC_URI = "file://ap64350.c \
        file://Makefile \
"

S = "${WORKDIR}"

# OE build directives
EXTRA_OEMAKE:append:task-install = " -C ${STAGING_KERNEL_DIR} M=${S}"
EXTRA_OEMAKE += "KDIR=${STAGING_KERNEL_DIR}"

# Autoinstall (optionally disable)
KERNEL_MODULE_AUTOLOAD += "ap64350"