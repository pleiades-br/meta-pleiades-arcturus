DESCRIPTION = "Arcturus String to synthesis speech "
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "git://github.com/pleiades-br/arcturus-espeak.git;protocol=https;branch=main"
SRCREV = "3cd351fa0aab44f63c3b5d9fadfad5f782119ee0"

S = "${WORKDIR}/git"

inherit setuptools3 

do_install:append () {
    install -d ${D}${bindir} 
    install -m 0755 str2speak ${D}${bindir}
}

RDEPENDS:${PN} += " espeak"