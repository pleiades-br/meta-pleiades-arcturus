FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += " \
    file://lighttpd.conf \
"

RDEPENDS:${PN}:append = " 
    lighttpd-module-rewrite \
    lighttpd-module-setenv \
"