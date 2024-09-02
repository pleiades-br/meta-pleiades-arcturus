#
# IC GPIO Initialization driver
#

# Bitbake class(es)
inherit module 

# Dependencies
DEPENDS = "virtual/kernel"

# Metadata
SUMMARY = "Arcturus GPIO control for board ICs"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

SRC_URI = "file://arcturus_gpio_ctrl.c \
        file://Makefile \
"

S = "${WORKDIR}"

# OE build directives
EXTRA_OEMAKE:append:task-install = " -C ${STAGING_KERNEL_DIR} M=${S}"
EXTRA_OEMAKE += "KDIR=${STAGING_KERNEL_DIR}"

# Autoinstall (optionally disable)
KERNEL_MODULE_AUTOLOAD += "arcturus_gpio_ctrl"