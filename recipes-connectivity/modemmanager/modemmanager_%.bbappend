PACKAGECONFIG =  "vala mbim qmi at \
    ${@bb.utils.filter('DISTRO_FEATURES', 'systemd polkit', d)} \
"

