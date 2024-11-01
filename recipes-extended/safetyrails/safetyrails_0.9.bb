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
    "
SRCREV = "14e6723eb829847968573f866f71d2a5c9061524"

S = "${WORKDIR}/git"

inherit setuptools3

do_install:append () {
    install -d ${D}${bindir}
    install -m 0755 sftrails ${D}${bindir}

    install -d ${D}${sysconfdir}/sftrails/
    install -m 0644 ${WORKDIR}/sftrails.conf ${D}${sysconfdir}/sftrails/
    
    install -d ${D}/opt/sftrails/
    install -m 0666 ${WORKDIR}/pre_dtmf.wav ${D}/opt/sftrails/
    install -m 0666 ${WORKDIR}/pos_dtmf.wav ${D}/opt/sftrails/
    install -m 0666 ${WORKDIR}/text_output.wav ${D}/opt/sftrails/
}

RDEPENDS:${PN} += " python3-numpy python3-gpiod python3-paho-mqtt espeak"
FILES:${PN} += "/opt/sftrails/*"
