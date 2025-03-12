DESCRIPTION = "udev rules for Canopus"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI:append:plds-myd-y6ull = "\
    file://78-mm-arcturus.rules \
    file://60-mm-imx-uart-whitelist.rules \
    file://9-arcturus-gpio.rules \
"


do_install () {
    install -d ${D}${sysconfdir}/udev/rules.d
    install -m 0644 ${WORKDIR}/60-mm-imx-uart-whitelist.rules ${D}${sysconfdir}/udev/rules.d/
    install -m 0644 ${WORKDIR}/78-mm-arcturus.rules ${D}${sysconfdir}/udev/rules.d/
    install -m 0644 ${WORKDIR}/79-arcturus-gpio.rules ${D}${sysconfdir}/udev/rules.d/
}