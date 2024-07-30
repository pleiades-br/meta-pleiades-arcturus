FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += " \
    file://wired.network \
    file://wlan.network \
    "

FILES:${PN} += " \
    ${sysconfdir}/systemd/network/wired.network \
    ${sysconfdir}/systemd/network/wlan.network \
    "

PACKAGECONFIG:append = " networkd iptc"
PACKAGECONFIG[acl] = "-Dacl=true,-Dacl=false,acl"

do_install:append() {
    install -d ${D}${sysconfdir}/systemd/network
    install -m 0644 ${WORKDIR}/wired.network ${D}${sysconfdir}/systemd/network
    #install -m 0644 ${WORKDIR}/wlan.network ${D}${sysconfdir}/systemd/network
}

FILES_${PN} += " \
    ${nonarch_base_libdir}/systemd/network \
"