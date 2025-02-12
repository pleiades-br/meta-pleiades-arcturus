FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

IMAGE_INSTALL:append = " safetyrails"
IMAGE_INSTALL:append = " udev-arcturus-rules"
IMAGE_INSTALL:append = " ruart"
IMAGE_INSTALL:append = " picocom"
IMAGE_INSTALL:append = " lighttpd"
IMAGE_INSTALL:append = " arcturus-web"
IMAGE_INSTALL:append = " arcturus-web-be"
IMAGE_INSTALL:append = " arcturus-str2dtmf"
IMAGE_INSTALL:append = " arcturus-str2speak"
IMAGE_INSTALL:append = " km-arcturus-gpio-ctrl"
IMAGE_INSTALL:append = " socat"

