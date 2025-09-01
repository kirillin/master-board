

# Сборка образа
docker build -t esp-builder .

# Запуск с доступом к USB и терминалу
docker run -it --rm \
    --privileged \
    -v /dev:/dev \
    -v $(pwd)/../:/workspace \
    esp-builder shell

# Или для прямого выполнения команд
docker run -it --rm \
    --privileged \
    -v /dev:/dev \
    -v $(pwd):/workspace \
    esp-builder make flash



cd /opt/esp/idf/
git checkout 8d1a9c0
git submodule update --init --recursive
bash add_path.sh 
/usr/bin/python -m pip install --user -r /opt/esp/idf/requirements.txt
cd /workspace/firmware/
make monitor


root@fa16d2f41959:/workspace/firmware# history 
    1  xtensa-esp32-elf-gcc --version
    2  cd /opt/esp/
    3  l
    4  ./entrypoint.sh 
    5  xtensa-esp32-elf-gcc --version
    6  l
    7  cd idf/
    8  l
    9  ./install.sh 
   10  . ./export.
   11  . ./export.sh 
   12  git checkout 8d1a9c0
   13  \
   14  git submodule update --init --recursive
   15  bash add_path.sh
   16  /usr/bin/python -m pip install --user -r /opt/esp/idf/requirements.txt
   17  cd /workspace/firmware/
