LOCAL_PATH=/home/robot/
SOURCE=main.cpp
SOURCE2=robot.hpp
REMOTE_PATH="robot/"
ADDRESS=brand17@192.168.0.30
OUTPUT=a.out
#ssh "${ADDRESS}" cd robot/remote_libs | ls
#ssh "${ADDRESS}" ls
scp "${LOCAL_PATH}${SOURCE}" "${ADDRESS}:${REMOTE_PATH}${SOURCE}"
scp "${LOCAL_PATH}${SOURCE2}" "${ADDRESS}:${REMOTE_PATH}${SOURCE2}"
ssh "${ADDRESS}" aarch64-linux-gnu-g++ -g "${REMOTE_PATH}${SOURCE}" -o "${REMOTE_PATH}${OUTPUT}"\
    -Irobot/include/eigen-3.4.0 -Irobot/include -Irobot -Lrobot/remote_libs -lwiringPi -lcrypt -lrt -march=armv8-a
scp "${ADDRESS}:${REMOTE_PATH}${OUTPUT}" "${LOCAL_PATH}${OUTPUT}"
chmod 777 "${LOCAL_PATH}${OUTPUT}"