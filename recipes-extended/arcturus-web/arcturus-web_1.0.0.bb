DESCRIPTION = "Arcturus web configuration page"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "https://github.com/pleiades-br/arcturus-web/archive/refs/heads/main.zip;protocol=http;"
SRC_URI[md5sum] = "b9459e3ed9561b2e122b0c5943c5de22"
SRC_URI[sha256sum] = "eaf5d2b1be17379bcbe92fce64675dbf84029fe0ca028a591ba820c85860251b"

RDEPENDS:${PN} = " lighttpd"

S = "${WORKDIR}"

MY_DESTINATION = "/www/pages"

do_install(){
   install -d ${D}${MY_DESTINATION}
   cp -r ${S}/arcturus-web-main/* ${D}${MY_DESTINATION}
}

FILES:${PN} += "/www"
FILES:${PN} += "/www/pages"
FILES:${PN} += "${MY_DESTINATION}/*"