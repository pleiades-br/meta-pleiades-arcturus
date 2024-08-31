FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append:plds-myd-y6ull = " \
    file://dts/myb-imx6ull-14x14-base-arcturus.dts \
    file://dts/myd-y6ull-emmc-arcturus.dts \
    file://dts/myd-y6ull-gpmi-weim-arcturus.dts \
    file://pac194x/pac194x.c \
    file://pac194x/microchip,pac194x.yaml \
    file://defconfig_arcturus"

SRCREV="50912be386017c8d2ca7f0c9c0a32fa7ac84a283"

IMX_KERNEL_CONFIG_AARCH32 = "defconfig_arcturus"

do_unpack:append:plds-myd-y6ull() {
    bb.build.exec_func('copy_arcturus_files', d)
}

copy_arcturus_files() {
    cp -f dts/myb-imx6ull-14x14-base-arcturus.dts dts/myd-y6ull-emmc-arcturus.dts dts/myd-y6ull-gpmi-weim-arcturus.dts ${S}/arch/arm/boot/dts
    cp -f defconfig_arcturus ${S}/arch/arm/configs/
}

DEPENDS += " firmware-imx"


