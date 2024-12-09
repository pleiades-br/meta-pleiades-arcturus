DESCRIPTION = "Arcturus web configuration page"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "git://github.com/pleiades-br/arcturus-web.git;protocol=http;branch=main"
SRCREV="30486fa7b6768b96b042ede4467498c96fd879f0"

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