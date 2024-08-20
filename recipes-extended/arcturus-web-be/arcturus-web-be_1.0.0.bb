DESCRIPTION = "Arcturus web configuration page"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"


SRC_URI = "git://github.com/pleiades-br/arcturus-web-backend.git;protocol=https;branch=main"
SRCREV = "23758b8457ed3bf4867387243d4a1731e2951ca6"

S = "${WORKDIR}/git"

inherit setuptools3

do_install:append () {
    install -d ${D}${bindir}
    install -m 0755 arcturus-web-be.py ${D}${bindir}
}