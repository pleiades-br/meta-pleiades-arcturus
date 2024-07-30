FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += " \
    file://wired.nmconnection \
    "

FILES:${PN} += " \
    ${sysconfdir}/NetworkManager/system-connections/wired.nmconnection \
    "

do_install:append() {
    install -d ${D}${sysconfdir}/NetworkManager/system-connections
    install -m 0644 ${WORKDIR}/wired.nmconnection ${D}${sysconfdir}/NetworkManager/system-connections
}

FILES_${PN} += " \
    ${nonarch_base_libdir}/NetworkManager/system-connections \
"