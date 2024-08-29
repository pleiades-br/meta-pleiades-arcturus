DESCRIPTION = "Arcturus String 2 DTMF script"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "git://github.com/pleiades-br/arcturus-str2dtmf.git;protocol=https;branch=main"
SRCREV = "2f62c6663bdff131f419bd352a1a3f23235cab01"

S = "${WORKDIR}/git"

inherit setuptools3 

do_install:append () {
    install -d ${D}${bindir} 
    install -m 0755 str2dtmf ${D}${bindir}
}

RDEPENDS:${PN} += " python3-numpy"