#sudo  modprobe -r gs_usb

#sudo modprobe can
#sudo modprobe can_raw
#sudo modprobe can_dev

#sudo insmod gs_usb.ko

#sudo chmod -R 777 /dev/ttyACM*

make

sudo modprobe -r gs_usb


sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev



sudo insmod gs_usb.ko


sudo ip link set can1 down
sudo ip link set can2 down
sudo ip link set can1 txqueuelen 1000
sudo ip link set can2 txqueuelen 1000

sudo ip link set can1 type can \
    bitrate 1000000 \
    dbitrate 5000000 \
    sample-point 0.75 \
    dsample-point 0.875 \
    restart-ms 100 \
    fd on

sudo ip link set can2 type can \
    bitrate 1000000 \
    dbitrate 5000000 \
    sample-point 0.75 \
    dsample-point 0.875 \
    restart-ms 100 \
    fd on

sudo ip link set can1 up
sudo ip link set can2 up


#source devel/setup.bash
