# HOW TO CONNECT SSH INTO RASPBERRYPI:

### OLD INFO
user: ethan.storer
hostname: raspberrypi
password: valporobotics

### NEW INFO
user: nbeals
hostname: raspberrypi
password: valporobotics


## to connect:
### Wires:
run Ethernet from computer to pi
run microUSB from power supply to pi
run USB from PS controller to pi
run HDMI from computer to pi _optional_
run keyboard to pi _optional_

### SSH:
wait for pi to boot
open command line and run "ping hostname.local -t"
open VS code
open remote window -> connect to host -> hostname.local -> enter password

## learning to navigate pi terminal (learning linux):
[Youtube link to linux terminal course](https://www.youtube.com/playlist?list=PLGs0VKk2DiYypuwUUM2wxzcI9BJHK4Bfh)

[Linux commands](https://www.geeksforgeeks.org/linux-commands/)


## setting up blank pi for testing:
### Make sure PI is up to date:
in ssh terminal
    sudo apt update
    sudo apt upgrade

### Bluetooth:
install libusb-dev package
    sudo apt install libusb-dev

download and make folder for sixpair code
    mkdir ~/sixpair
    cd ~/sixpair
    wget http://www.pabr.org/sixlinux/sixpair.c

compile sixpair program
    gcc -o sixpair sixpair.c -lusb


see: [Connecting Playstation controllers to raspberry pi]()

### CAN:

see [Adding CAN to the Raspberry PI]()