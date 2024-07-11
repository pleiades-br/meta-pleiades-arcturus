FILESEXTRAPATHS:prepend := "${THISDIR}/${PN}:"

SRC_URI += " \
    file://0001-fix-Adding-ifdef-config-to-avoid-unusable-variables.patch;patchdir=../.."