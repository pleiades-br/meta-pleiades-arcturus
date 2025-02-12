FILESEXTRAPATHS:prepend := "${THISDIR}/files:"


SRC_URI:append:plds-myd-y6ull  += " \
    file://socat-eg91-at.service \
    file://socat-eg91-gps.service \ 
"

EXTRA_OECONF += "ac_cv_have_z_modifier=yes sc_cv_sys_crdly_shift=9"

do_install:append() {
    if ${@bb.utils.contains('DISTRO_FEATURES', 'systemd', 'true', 'false', d)}; then
        install -d ${D}${systemd_system_unitdir}
        install -m 0644 ${WORKDIR}/socat-eg91-at.service ${D}${systemd_system_unitdir}/
        install -m 0644 ${WORKDIR}/socat-eg91-gps.service ${D}${systemd_system_unitdir}/
    fi
}

SYSTEMD_SERVICE:${PN} = "socat-eg91-at.service socat-eg91-gps.service"
SYSTEMD_AUTO_ENABLE = "enable"