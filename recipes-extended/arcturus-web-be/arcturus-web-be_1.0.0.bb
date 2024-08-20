DESCRIPTION = "Arcturus web configuration page"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"


SRC_URI = "git://github.com/pleiades-br/arcturus-web-backend.git;protocol=https;branch=main"
SRCREV = "050c6a821488f045ab94ccc0d21f56df432bae7b"

S = "${WORKDIR}/git"

inherit setuptools3

do_install:append () {
    install -d ${D}${bindir}
    install -m 0755 arcturus-web-be.py ${D}${bindir}
}