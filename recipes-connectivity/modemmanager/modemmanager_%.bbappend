FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI:append = " \
    file://mm-startup-cmds.service \
"

PACKAGECONFIG =  "vala mbim qmi at \
    ${@bb.utils.filter('DISTRO_FEATURES', 'systemd polkit', d)} \
"

inherit systemd

do_install:append() {
    if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
        install -d ${D}${systemd_system_unitdir}
        install -m 0644 ${WORKDIR}/mm-startup-cmds.service ${D}${systemd_system_unitdir}/
    fi
}

SYSTEMD_SERVICE:${PN} = " \
    mm-startup-cmds.service \
"
SYSTEMD_AUTO_ENABLE = "enable"

FILES:${PN} += " ${systemd_system_unitdir}"