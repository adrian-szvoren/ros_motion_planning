x=(-9.0 9.0)
y=(-3.0 3.0)

for i in $(seq 0 $((${#x[*]}-1)))
do
    ./main.sh &
    sleep 10s

    rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "{header: {stamp: now, frame_id: "map"}, pose: {position: {x: ${x[i]}, y: ${y[i]}, z: 0.0}, orientation: {w: 1.0}}}" &
    sleep 2m
    
    ./killpro.sh
    sleep 20s
done
