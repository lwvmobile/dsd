#! /bin/bash
#
#run cygwin installer with this to get all the packages
#use http://www.gtlib.gatech.edu mirror (seems relatively fast)
#.\setup-x86-64.exe --packages nano,libpulse-devel,libpulse-mainloop-glib0,libpulse-simple0,libpulse0,pulseaudio,pulseaudio-debuginfo,pulseaudio-equalizer,pulseaudio-module-x11,pulseaudio-module-zeroconf,pulseaudio-utils,sox-fmt-pulseaudio,libusb0,libusb1.0,libusb1.0-debuginfo,libusb1.0-devel,libncurses++w10,libncurses-devel,libncursesw10,ncurses,cmake,gcc-core,gcc-debuginfo,gcc-objc,git,make,socat,sox,sox-fmt-ao,unzip,wget,gcc-g++,libsndfile-devel
#Nurse the installer by clicking next and waiting for it to finish, etc
#Ignore the warning popup telling you to install libusb from sourceforge, it is NOT NEEDED and actually makes the dongle not work with DSD-FME

cdir=$(pwd)
clear
printf "Digital Speech Decoder: Florida Man Edition - Auto Installer For Cygwin\n
This will install the required packages, clone, build, and install DSD-FME only.
This has been tested on Cygwin x86-64 as of 20250211. This may take a while!\n
MBELib is considered a requirement on this build.
You must view the Patent Notice prior to continuing.
The Patent Notice can be found at the site below.
https://github.com/lwvmobile/mbelib#readme
Please confirm that you have viewed the patent notice by entering y below.\n\n"
read -p "Have you viewed the patent notice? y/N " ANSWER
ANSWER=$(printf "$ANSWER"|tr '[:upper:]' '[:lower:]')
if [ "$ANSWER" = "y" ]; then

  #is this needed?
  LD_LIBRARY_PATH=/usr/local/lib
  export LD_LIBRARY_PATH
  echo $LD_LIBRARY_PATH

  #ITPP
  cd $cdir
  printf "Installing itpp 4.3.1 from source http://sourceforge.net/projects/itpp/files/latest/download?source=files\n Please wait!\n"
  wget -O itpp-latest.tar.bz2 http://sourceforge.net/projects/itpp/files/latest/download?source=files
  tar xjf itpp-latest.tar.bz2
  cd itpp-4.3.1/
  mkdir build
  cd build
  cmake ..
  make
  make install
  #can't remember if copying this to /bin is required or not
  #cp cygitpp-8.dll /bin

  #MBELIB
  cd $cdir
  git clone https://github.com/lwvmobile/mbelib.git
  cd mbelib
  git checkout ambe_tones
  mkdir build
  cd build
  cmake ..
  make
  make install
  cp cygmbe-1.dll /bin

  #CODEC2
  cd $cdir
  printf "Installing CODEC2\n Please wait!\n"
  git clone https://github.com/drowe67/codec2.git
  cd codec2
  mkdir build
  cd build
  cmake ..
  make
  make install

  #RTL-SDR
  cd $cdir
  printf "Installing RTL-SDR\n Please wait!\n"
  git clone https://github.com/lwvmobile/rtl-sdr.git
  cd rtl-sdr
  mkdir build
  cd build
  cmake ..
  make
  make install

  #DSD-FME
  cd $cdir
  printf "Installing RTL-SDR\n Please wait!\n"
  git clone https://github.com/lwvmobile/dsd-fme.git
  cd dsd-fme
  git checkout cygwin_fixes
  mkdir build
  cd build
  cmake ..
  make
  make install

  printf "Any issues, Please report to either:\nhttps://github.com/lwvmobile/dsd-fme/issues or\nhttps://forums.radioreference.com/threads/dsd-fme.438137/\n\n"

else
  printf "Sorry, you cannot build DSD-FME without acknowledging the Patent Notice.\n\n"
fi