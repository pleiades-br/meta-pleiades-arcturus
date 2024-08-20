DESCRIPTION = "Arcturus web configuration page"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"


SRC_URI = "git://github.com/pleiades-br/arcturus-web-backend.git;protocol=https;branch=main"
SRCREV = "1a84ff9c005b0bb7eb7df4016fd132c3e5e650fd"

S = "${WORKDIR}/git"

inherit setuptools3

do_install:append () {
    install -d ${D}${bindir}
    install -m 0755 arcturusWebBE.py ${D}${bindir}
}