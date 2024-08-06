SUMMARY = "PPP config file for Quectel EG91"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

RDEPENDS:${PN} = "ppp"

SRC_URI += " \
    file://eg91-dial \
    file://eg91-lte-modem \
    file://interfaces \
    "

S = "${WORKDIR}"

do_install () {
    install -d ${D}${sysconfdir}/ppp/chatscripts
    install -d ${D}${sysconfdir}/ppp/peers
    install -d ${D}${sysconfdir}/network
    install -m 0755 eg91-dial ${D}${sysconfdir}/ppp/peers/
    install -m 0755 eg91-lte-modem ${D}${sysconfdir}/ppp/chatscripts/
    install -m 0755 interfaces ${D}${sysconfdir}/network/
}

FILES:${PN} += "${sysconfdir}/ppp/"
FILES:${PN} += "${sysconfdir}/ppp/peers/"
FILES:${PN} += "${sysconfdir}/ppp/chatscripts"
FILES:${PN} += "${sysconfdir}/network"