first:
docker compose build


then:
chmod +x start.sh

./start.sh

docker compose run --rm 

inside docker: 

    colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.sh
    export GZ_VERBOSE=1
    clear
    ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true


rosdep install --from-paths src -y --ignore-src --skip-keys "mock_components"

colcon build --symlink-install

mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone -b humble  https://github.com/moveit/moveit2_tutorials
vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos
sudo apt remove ros-humble-moveit*
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro humble -y
cd ~/ws_moveit
colcon build --mixin release
source ~/ws_moveit/install/setup.bash
 echo 'source ~/ws_moveit/install/setup.bash' >> ~/.bashrc