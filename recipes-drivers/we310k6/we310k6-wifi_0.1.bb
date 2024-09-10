inherit deploy autotools

DESCRIPTION = "RTL8852 Wireless LAN Driver"
LICENSE     = "CLOSED"
LIC_FILES_CHKSUM = ""

SRC_URI = "file://we310k6.tar"
S = "${WORKDIR}/we310k6/WIFI/driver"
F = "${WORKDIR}/we310k6/WIFI/firmware"

do_configure () {
	sed -i 's/CONFIG_PLATFORM_I386_PC = y/CONFIG_PLATFORM_IMX6 = y/' ${S}/Makefile
}
do_compile () {
	cd ${S}/
	oe_runmake
}

do_install () {
	install -d ${D}/lib
    install -d ${D}/lib/modules
    install -m 0777 ${S}/we310k6.ko           ${D}/lib/modules/
	install -d ${D}/lib/firmware
	install -m 0777 ${F}/*.txt    ${D}/lib/firmware
}

do_clean_drv () {
	cd ${S}/
	unset LDFLAGS
	oe_runmake clean
}

addtask clean_drv before do_clean

EXTRA_OEMAKE += "KDIR=${STAGING_KERNEL_DIR}"

FILES:${PN} += "/lib/*"

