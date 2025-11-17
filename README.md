# thesis-fw

Using ESP32-S2-DEVKITC-1U-N8R2 board

On MacOS setup steps:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html

1. Prerequisite software packages:
brew install cmake ninja dfu-util

also install ccache for faster builds:
brew install ccache

make sure python3 is installed:
brew install python3

2. install ESP-IDF:
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git

3. Setup ESP-IDF tools
cd ~/esp/esp-idf
./install.sh esp32s2

install compilation tools needed by ESP-IDF:
export IDF_TOOLS_PATH="$HOME/required_idf_tools_path"
./install.sh

. ./export.sh

4. Setup Environment Variables
In terminal where you are going to use ESP-IDF:
. $HOME/esp/esp-idf/export.sh

can setup an alias:
alias get_idf='. $HOME/esp/esp-idf/export.sh'

5. Example project: Hello World
cp -r $IDF_PATH/examples/get-started/hello_world .

establish serial connection: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/get-started/establish-serial-connection.html

check for COM port (macOS):
ls /dev/cu.*

(make sure cable supports data transfer)

configure project:
cd _path_/hello_world
idf.py set-target esp32s2
idf.py menuconfig

in the menu (Component config > ESP System Settings > Channel for console output), set output to USB CDC

build project:
idf.py build

flash onto device:
idf.py -p PORT flash
where PORT found from before
(flash automatically builds and flashes)

monitor output:
idf.py -p PORT monitor
(this opens the IDF Monitor application: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/api-guides/tools/idf-monitor.html)

can change crystal xtal frequency:
menuconfig: Component config > Hardware Settings > Main XTAL > Config > Main XTAL frequency

