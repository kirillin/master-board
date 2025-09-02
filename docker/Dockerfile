FROM espressif/idf:v4.0.4

RUN apt-get update && apt-get install -y \
    udev \
    dfu-util \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="303a", ATTR{idProduct}=="00??", MODE="0666"' > /etc/udev/rules.d/99-esp.rules && \
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="00??", MODE="0666"' >> /etc/udev/rules.d/99-esp.rules

RUN pip3 install esptool pyserial

RUN cd /opt/esp/idf/ && ./install.sh &&  . ./export.sh && /usr/bin/python -m pip install --user -r /opt/esp/idf/requirements.txt && git checkout 8d1a9c0 && git submodule update --init --recursive
RUN cd /opt/esp/idf/ && bash add_path.sh

WORKDIR /workspace

# Copy entrypoint and setup scripts
COPY entrypoint.sh /entrypoint.sh
COPY setup_esp_idf.bash /setup_esp_idf.bash
RUN chmod +x /entrypoint.sh /setup_esp_idf.bash

ENTRYPOINT ["/entrypoint.sh"]