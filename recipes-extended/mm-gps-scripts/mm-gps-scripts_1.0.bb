DESCRIPTION = "GPS scripts that will take care of modemmanager if the lte has only one uart \
    connection "
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"
FILESEXTRAPATHS:prepend := "${THISDIR}/files:"


SRC_URI += " \
    file://mm-startup-cmds.service \
    file://mm-gps-get.sh \
    file://mm-gps-start.sh \
"

inherit systemd

do_install() {
    if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
        install -d ${D}${systemd_system_unitdir}
        install -m 0644 ${WORKDIR}/mm-startup-cmds.service ${D}${systemd_system_unitdir}/
    fi

    # Install the GPS data collection script
    install -d ${D}${bindir}
    install -m 0755 ${WORKDIR}/mm-gps-get.sh ${D}${bindir}/    
    install -m 0755 ${WORKDIR}/mm-gps-start.sh ${D}${bindir}/
}

SYSTEMD_SERVICE:${PN} += " \
    mm-startup-cmds.service \
"

RDEPENDS:${PN} += "bash modemmanager"

FILES:${PN} += " ${systemd_system_unitdir}"