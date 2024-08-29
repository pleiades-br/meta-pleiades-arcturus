DESCRIPTION = "Arcturus String 2 DTMF script"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "git://github.com/pleiades-br/arcturus-str2dtmf.git;protocol=https;branch=main"
SRCREV = "eaacb9fc6f8b7a2e9f516d89ee29ad08c8f74e6b"

S = "${WORKDIR}/git"

inherit setuptools3 

do_install:append () {
    install -d ${D}${bindir} 
    install -m 0755 str2dtmf ${D}${bindir}
}

RDEPENDS:${PN} += " python3-numpy"