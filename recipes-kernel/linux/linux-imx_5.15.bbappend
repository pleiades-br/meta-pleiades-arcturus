FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI:append:plds-myd-y6ull = " \
    file://dts/myb-imx6ull-14x14-base-arcturus.dts \
    file://dts/myd-y6ull-emmc-arcturus.dts \
    file://dts/myd-y6ull-gpmi-weim-arcturus.dts \
    file://pac194x/pac194x.c \
    file://pac194x/microchip,pac194x.yaml \
    file://pac194x/0001-adding-pac194x-adc.patch \
    file://ads122c04/ti-ads122c04.c \
    file://ads122c04/ti,ads122c04.yaml \
    file://ads122c04/0001-adding-ads122c04-driver.patch \
    file://0001-changing-arm-for-hard-float.patch \
    file://defconfig_arcturus"

SRCREV="50912be386017c8d2ca7f0c9c0a32fa7ac84a283"

IMX_KERNEL_CONFIG_AARCH32 = "defconfig_arcturus"

do_unpack:append:plds-myd-y6ull() {
    bb.build.exec_func('copy_arcturus_files', d)
}

copy_arcturus_files() {
    cp -f dts/myb-imx6ull-14x14-base-arcturus.dts dts/myd-y6ull-emmc-arcturus.dts dts/myd-y6ull-gpmi-weim-arcturus.dts ${S}/arch/arm/boot/dts
    cp -f defconfig_arcturus ${S}/arch/arm/configs/
    cp -f pac194x/pac194x.c ${S}/drivers/iio/adc/
    cp -f pac194x/microchip,pac194x.yaml ${S}/Documentation/devicetree/bindings/iio/adc/
    cp -f ads122c04/ti-ads122c04.c ${S}/drivers/iio/adc/
    cp -f ads122c04/ti,ads122c04.yaml ${S}/Documentation/devicetree/bindings/iio/adc/
}

DEPENDS += " firmware-imx"


