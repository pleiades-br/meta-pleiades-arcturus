DESCRIPTION = "Arcturus web configuration page"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://LICENSE;md5=8e66d58e4d5ff0650d10b76e6c39c852"

SRC_URI += "https://github.com/pleiades-br/arcturus-web/archive/refs/heads/main.zip;protocol=http"
SRC_URI[md5sum] = "37e7c1303db943e56e603e9ec699235b"
SRC_URI[sha256sum] = "e86ee36e8716fe4b33be9bcb272ef9835c5e5ec3b58fd3e2c681334175945e4b"

RDEPENDS:${PN} = "lighttpd"

S = "${WORKDIR}/${PN}-${PV}"

MY_DESTINATION = "/www/pages"

do_install(){
   install -d ${D}${MY_DESTINATION}
   cp -r ${S}/* ${D}${MY_DESTINATION}
}

FILES_${PN} += "${MY_DESTINATION}/*"