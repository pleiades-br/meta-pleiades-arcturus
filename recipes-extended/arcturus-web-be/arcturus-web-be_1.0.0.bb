DESCRIPTION = "Arcturus web configuration page"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"


SRC_URI = " \
            git://github.com/pleiades-br/arcturus-web-backend.git;protocol=https;branch=main \
            file://arcwebbe \
        "
SRCREV = "b6bb84659f219d9f97374ae7ff16540f8fc1fee1"


S = "${WORKDIR}/git"

inherit setuptools3 systemd

SYSTEMD_SERVICE:${PN} = "arcwebbe.service"

INITSCRIPT_NAME = "arcwebbe"
INITSCRIPT_PARAMS = "defaults 80"

do_install:append () {
    install -d ${D}${bindir} 
    install -m 0755 arcwebbe ${D}${bindir}

    install -d ${D}${sysconfdir}/init.d
    install -m 0755 ${WORKDIR}/arcwebbe ${D}${sysconfdir}/init.d

    install -d ${D}${systemd_system_unitdir}
	install -m 0644 ${S}/doc/systemd/arcwebbe.service ${D}${systemd_system_unitdir}
    sed -i -e 's,@SBINDIR@,${sbindir},g' \
		-e 's,@SYSCONFDIR@,${sysconfdir},g' \
		-e 's,@BASE_BINDIR@,${base_bindir},g' \
		${D}${systemd_system_unitdir}/arcwebbe.service
}

FILES:${PN} += " ${systemd_system_unitdir}"
RDEPENDS:${PN} += " python3-netifaces"