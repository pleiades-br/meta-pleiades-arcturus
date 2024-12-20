DESCRIPTION = "Application in python for train derailment detection through GPIO analysis"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=4a4dcba7e9f35ff16fd3c325ea239fd6"
FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI = " \
    git://github.com/pleiades-br/arcturus-safetyRails.git;protocol=https;branch=main \
    file://sftrails.conf \
    file://pre_dtmf.wav \
    file://pos_dtmf.wav \
    file://text_output.wav \
    file://sftrails_service \
    "
SRCREV = "f52deb278f1d318eff3bcb696614f4c9fdee9437"

S = "${WORKDIR}/git"

inherit setuptools3 systemd

SYSTEMD_SERVICE:${PN} = "sftrails.service"

SYSTEMD_SERVICE:${PN} = "sftrails.service"
INITSCRIPT_NAME = "sftrails_service"
INITSCRIPT_PARAMS = "defaults 70"

do_install:append () {
    install -d ${D}${bindir}
    install -m 0755 sftrails ${D}${bindir}

    install -d ${D}${sysconfdir}/sftrails/
    install -m 0644 ${WORKDIR}/sftrails.conf ${D}${sysconfdir}/sftrails/
    
    install -d ${D}/opt/sftrails/
    install -m 0666 ${WORKDIR}/pre_dtmf.wav ${D}/opt/sftrails/
    install -m 0666 ${WORKDIR}/pos_dtmf.wav ${D}/opt/sftrails/
    install -m 0666 ${WORKDIR}/text_output.wav ${D}/opt/sftrails/

    install -d ${D}${sysconfdir}/init.d
    install -m 0755 ${WORKDIR}/sftrails_service ${D}${sysconfdir}/init.d

    install -d ${D}${systemd_system_unitdir}
	install -m 0644 ${S}/doc/systemd/sftrails.service ${D}${systemd_system_unitdir}
    sed -i -e 's,@SBINDIR@,${sbindir},g' \
		-e 's,@SYSCONFDIR@,${sysconfdir},g' \
		-e 's,@BASE_BINDIR@,${base_bindir},g' \
		${D}${systemd_system_unitdir}/sftrails.service
}

RDEPENDS:${PN} += " python3-numpy python3-gpiod python3-paho-mqtt espeak"
FILES:${PN} += "/opt/sftrails/*"
FILES:${PN} += " ${systemd_system_unitdir}"
