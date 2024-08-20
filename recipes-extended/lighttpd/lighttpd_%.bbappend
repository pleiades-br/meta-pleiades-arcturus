FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += " \
    file://lighttpd.conf \
"

RRECOMMENDS:${PN} = " \
    lighttpd-module-access \
    lighttpd-module-accesslog \
    lighttpd-module-proxy \
"