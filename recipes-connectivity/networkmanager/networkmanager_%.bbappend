FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += " \
    file://wired.nmconnection \
    file://NetworkManager.conf \
    "

FILES:${PN} += " \
    ${sysconfdir}/NetworkManager/system-connections/wired.nmconnection \
    ${sysconfdir}/NetworkManager/NetworkManager.conf \
    "

do_install:append() {
    install -d ${D}${sysconfdir}/NetworkManager/system-connections
    install -m 0644 ${WORKDIR}/wired.nmconnection ${D}${sysconfdir}/NetworkManager/system-connections
    install -m 0644 ${WORKDIR}/NetworkManager.conf ${D}${sysconfdir}/NetworkManager
}

FILES_${PN} += " \
    ${nonarch_base_libdir}/NetworkManager/system-connections \
    ${nonarch_base_libdir}/NetworkManager \
"