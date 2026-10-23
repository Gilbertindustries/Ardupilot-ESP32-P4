#!/usr/bin/env bash
# if you have modules/esp_idf setup as a submodule, then leave it as a submodule and switch branches

COMMIT="735507283d5b2f9fb363a1901172dbd9e847945d"
# ESP-IDF v5.5.4. Do not pin this back to the v5.3.1-era commit
# (cc3203dc4f0): the ESP32-P4 rev 3.x / ECO7 silicon on the Waveshare
# ESP32-P4-WIFI6 board needs the ECO7 support (esp32p4.rom.eco5.*.ld
# linker scripts, ROM build esp32p4-eco7-20260109) which only landed in
# 5.5.x. With 5.3.1 the second-stage bootloader dies in
# uart_ll_set_baudrate() -> bootloader_console_init() because the UART
# source clock reads as 0 on this chip revision, giving a WDT reset loop.
BRANCH="release/v5.5"

if [ ! -d modules ]; then
echo "this script needs to be run from the root of your repo, sorry, giving up."
exit 1
fi
echo `ls modules`
cd modules

if [ ! -d esp_idf ]; then
    echo 'did not find modules/esp_idf folder, making it.' ; 
    mkdir -p -v esp_idf
else
    echo 'found modules/esp_idf folder' ; 
fi

echo "looking for submodule or repo..."
if [ `git submodule | grep esp_idf | wc | cut -c1-7` == '1'  ]; then 
    echo "found real submodule, syncing"
    ../Tools/gittools/submodule-sync.sh >/dev/null
else
    echo "esp_idf is NOT a submodule"

    if  [ ! `ls  esp_idf/install.sh 2>/dev/null` ]; then
        echo "found empty IDF, cloning"
        # add esp_idf as almost submodule, depths  uses less space
        git clone -b "$BRANCH"  https://github.com/espressif/esp-idf.git esp_idf
        git checkout $COMMIT
    fi
fi

echo "inspecting possible IDF... "
cd esp_idf
echo `git rev-parse HEAD`
# this is the specific commit that represents the v5.5 branch of the esp_idf
if [ `git rev-parse HEAD` == "$COMMIT" ]; then
    echo "IDF version '$BRANCH' found OK, great.";
else
    echo "looks like an idf, but not $BRANCH, or wrong commit , trying to switch branch and reflect upstream";
    ../../Tools/gittools/submodule-sync.sh >/dev/null
    git fetch ; git checkout -f "$BRANCH"
    git checkout $COMMIT

    # retry same as above
    echo `git rev-parse HEAD`
    if [ `git rev-parse HEAD` == "$COMMIT" ]; then
        echo "IDF version '$BRANCH' found OK, great.";
        git checkout $COMMIT
    fi
fi
cd ../..

cd modules/esp_idf
git submodule update --init --recursive

echo
echo "installing missing python modules"
python3 -m pip install empy==3.3.4
python3 -m pip install pexpect
python3 -m pip install future

cd ../..

echo
echo "after changing IDF versions [ such as between 4.4 and 5.3 ] you should re-run these in your console:"
echo "./modules/esp_idf/install.sh"
echo "source ./modules/esp_idf/export.sh"
