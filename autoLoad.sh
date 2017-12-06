#! /bin/bash   
case "$1" in
    start)
        echo "Starting LED Blink"
        /home/jobsg/MOI_Robot_Winter/src/winter_socket/src/Serverrobot.py &
        ;;
    stop)
        echo "Stopping ledblink"
        #killall ledblink.py
        kill $(ps aux | grep -m 1 'python /home/jobsg/MOI_Robot_Winter/src/winter_socket/src/Serverrobot.py' | awk '{ print $2 }')
        ;;
    *)
        echo "Usage: service ledblink start|stop"
        exit 1
        ;;
esac
exit 0
#source /opt/ros/indigo/setup.bash
#chomd +x autoLoad.sh
#cp autoLoad.sh /etc/init.d
#update-rc.d start-zk.sh defaults
#cd /etc/init.d
#sudo update-rc.d -f autoLoad.sh remove
# gnome-session-properties
