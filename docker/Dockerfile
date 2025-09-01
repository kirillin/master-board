FROM espressif/idf:v4.0.4

# Install additional system dependencies (if needed)
RUN apt-get update && apt-get install -y \
    udev \
    dfu-util \
    && rm -rf /var/lib/apt/lists/*

# Set up udev rules for ESP32 devices
RUN echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="303a", ATTR{idProduct}=="00??", MODE="0666"' > /etc/udev/rules.d/99-esp.rules && \
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="00??", MODE="0666"' >> /etc/udev/rules.d/99-esp.rules

# Install additional Python tools
RUN pip3 install esptool pyserial

WORKDIR /workspace

# Copy entrypoint and setup scripts
COPY entrypoint.sh /entrypoint.sh
COPY setup_esp_idf.bash /setup_esp_idf.bash
RUN chmod +x /entrypoint.sh /setup_esp_idf.bash

ENTRYPOINT ["/entrypoint.sh"]