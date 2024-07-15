DESCRIPTION = "udev rules for Canopus"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI:append:plds-verdin-imx8mp-canopus = "\
    file://78-mm-canopus.rules \
"

do_install () {
    install -d ${D}${sysconfdir}/udev/rules.d
    install -m 0644 ${WORKDIR}/78-mm-canopus.rules ${D}${sysconfdir}/udev/rules.d/
}