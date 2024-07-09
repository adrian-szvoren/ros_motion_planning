ALGO=astar
MAP=warehouse
ATTACK=attack

wait_for_string() {
    local target_string="$1"

    grep --line-buffered -m 1 "$target_string" && echo "Found the target string: $target_string"
}

x=(-9.0 9.0)
y=(-3.0 3.0)

# rm ${FILE_TO_WATCH}

for i in $(seq 0 $((${#x[*]}-1))); do
    for n in {1..3}; do
        # ./main.sh > ${FILE_TO_WATCH} &
        # timeout 10s tail -f -n0 ${FILE_TO_WATCH} | grep -qe 'odom received!'

        # Start the command and pipe its output to grep, which will wait for the target string
        ./main.sh
        wait_for_string "odom received!"
        # while IFS= read -r line; do
        #     if [[ "$line" == *"$TARGET_STRING"* ]]; then
        #         echo "Found the target string: $TARGET_STRING"
        #         break
        #     fi
        # done
        # sleep 10s

        start=$(date +%s.%N)
        rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "{header: {stamp: now, frame_id: "map"}, pose: {position: {x: ${x[i]}, y: ${y[i]}, z: 0.0}, orientation: {w: 1.0}}}" &
        # sleep 2m
        timeout 2m tail -f -n0 ${FILE_TO_WATCH} | grep -qe 'GOAL Reached!'
        dur=$(echo "$(date +%s.%N) - $start" | bc)
        echo "${ALGO}, ${MAP}, ${ATTACK}, ${x[i]}, ${y[i]}, ${n}, ${dur}" >> data.csv

        ./killpro.sh
        timeout 20s tail -f -n0 ${FILE_TO_WATCH} | grep -qe 'shutting down processing monitor complete'
        # sleep 20s
    done
done
