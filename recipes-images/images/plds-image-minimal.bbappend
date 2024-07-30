FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

IMAGE_INSTALL:append = " safetyrails"
IMAGE_INSTALL:append = " udev-arcturus-rules"
IMAGE_INSTALL:append = " ruart"
IMAGE_INSTALL:append = " picocom"
IMAGE_INSTALL:append = " lighttpd"