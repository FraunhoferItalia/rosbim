
function rosbim() {
    case $1 in
    
    push)
        docker push fhi-git.fraunhofer.it:5050/construction-robotics/concert/rosbim:humble
        ;;

    empty)
        docker exec -it rosbim_docker terminator
        ;;

    build)
        docker compose build
        ;;

    *)
        echo "Usage: rosbim {empty|build}"
        ;;
    esac
}

complete -W "empty \ push \
            build " rosbim
