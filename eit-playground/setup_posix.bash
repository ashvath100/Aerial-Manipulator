#!/bin/bash

echo "Cleaning up"
find $PX4_DIR -name "*sdu_drone*" -delete

sed -i '/1003_sdu_drone.hil/d' $(echo $PX4_DIR)/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt

echo "Symlink"
#ln -s /home/$USER/vd_workspace/src/vd_gazebo/init.d-posix/* $PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/
ln -s $(echo $EIT_DIR)/src/eit-playground/init.d-posix/* $(echo $PX4_DIR)/ROMFS/px4fmu_common/init.d-posix/airframes/
ln -s $(echo $EIT_DIR)/src/eit-playground/init.d/* $(echo $PX4_DIR)/ROMFS/px4fmu_common/init.d/airframes/
ln -s $(echo $EIT_DIR)/src/eit-playground/mixers/* $(echo $PX4_DIR)/ROMFS/px4fmu_common/mixers/
ln -s $(echo $EIT_DIR)/src/eit-playground/models/* $(echo $PX4_DIR)/Tools/sitl_gazebo/models/
ln -s $(echo $EIT_DIR)/src/eit-playground/worlds/* $(echo $PX4_DIR)/Tools/sitl_gazebo/worlds/

echo "CMakeLists changes"
sed -i '/1002_standard_vtol.hil/a \ \ 1003_sdu_drone.hil' $(echo $PX4_DIR)/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt
