#!/bin/bash

sed -i '/1003_sdu_drone.hil/d' $(echo $PX4_DIR)/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt

echo "Symlink"
ln -s $(echo $MAS_DIR)/src/drone_with_arm/init.d-posix/* $(echo $PX4_DIR)/ROMFS/px4fmu_common/init.d-posix/airframes/
ln -s $(echo $MAS_DIR)/src/drone_with_arm/init.d/* $(echo $PX4_DIR)/ROMFS/px4fmu_common/init.d/airframes/
ln -s $(echo $MAS_DIR)/src/drone_with_arm/mixers/* $(echo $PX4_DIR)/ROMFS/px4fmu_common/mixers/
ln -s $(echo $MAS_DIR)/src/drone_with_arm/models/* $(echo $PX4_DIR)/Tools/sitl_gazebo/models/
ln -s $(echo $MAS_DIR)/src/drone_with_arm/worlds/* $(echo $PX4_DIR)/Tools/sitl_gazebo/worlds/
echo "CMakeLists changes"
sed -i '/1002_standard_vtol.hil/a \ \ 1003_sdu_drone.hil' $(echo $PX4_DIR)/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt
