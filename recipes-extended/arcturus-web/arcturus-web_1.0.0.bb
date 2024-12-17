DESCRIPTION = "Arcturus web configuration page"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "git://github.com/pleiades-br/arcturus-web.git;protocol=http;branch=main"
SRCREV="7047a74231c2d3b60b7f66e9b9f1010c9e913052"

RDEPENDS:${PN} = " lighttpd"

S = "${WORKDIR}"

MY_DESTINATION = "/www/pages"

do_install(){
   install -d ${D}${MY_DESTINATION}
   cp -r ${S}/git/* ${D}${MY_DESTINATION}
}

FILES:${PN} += "/www"
FILES:${PN} += "/www/pages"
FILES:${PN} += "${MY_DESTINATION}/*"