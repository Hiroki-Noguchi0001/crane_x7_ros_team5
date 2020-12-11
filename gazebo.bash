#!/bin/bash

# 実行方法
# $ chmod +x gazebo.bash
# $ ./gazebo.bash

USERNAME=$(whoami)

if [ -d "/home/$USERNAME/.gazebo/models" ]; then

    if [ -d "/home/$USERNAME/.gazebo/models/asstseal_model" ]; then
        cd /home/$USERNAME/.gazebo/models/asstseal_model
        echo "上田はんこモデル リポジトリを更新します"
        git pull
    else
        cd /home/$USERNAME/.gazebo/models
        echo "上田はんこモデル リポジトリをクローンします"
        git clone https://github.com/RobotDesign3-Team5/asstseal_model.git
    fi

    echo -n -e "\n"
    
    if [ -d "/home/$USERNAME/.gazebo/models/storage_model" ]; then
        cd /home/$USERNAME/.gazebo/models/storage_model
        echo "はんこ置きモデル リポジトリを更新します"
        git pull
    else
        cd /home/$USERNAME/.gazebo/models
        echo "はんこ置きモデル リポジトリをクローンします"
        git clone https://github.com/RobotDesign3-Team5/storage_model.git
    fi

    echo -n -e "\n"

    if [ -d "/home/$USERNAME/.gazebo/models/inkpad_model" ]; then
        cd /home/$USERNAME/.gazebo/models/inkpad_model
        echo "朱肉モデル リポジトリを更新します"
        git pull
    else
        cd /home/$USERNAME/.gazebo/models
        echo "朱肉モデル リポジトリをクローンします"
        git clone https://github.com/RobotDesign3-Team5/inkpad_model.git
    fi

    echo -n -e "\n"

    if [ -d "/home/$USERNAME/.gazebo/models/sealmat_model" ]; then
        cd /home/$USERNAME/.gazebo/models/sealmat_model
        echo "はんこモデル リポジトリを更新します"
        git pull
    else
        cd /home/$USERNAME/.gazebo/models
        echo "はんこモデル リポジトリをクローンします"
        git clone https://github.com/RobotDesign3-Team5/sealmat_model.git
    fi

    echo -n -e "\n"

    if [ -d "/home/$USERNAME/.gazebo/models/TissuePaper_model" ]; then
        cd /home/$USERNAME/.gazebo/models/TissuePaper_model
        echo "ティッシュペーパーモデル リポジトリを更新します"
        git pull
    else
        cd /home/$USERNAME/.gazebo/models
        echo "ティッシュペーパーモデル リポジトリをクローンします"
        git clone https://github.com/RobotDesign3-Team5/TissuePaper_model.git
    fi

    echo -n -e "\n"

    if [ -d "/home/$USERNAME/.gazebo/models/colorseal_model" ]; then
        cd /home/$USERNAME/.gazebo/models/colorseal_model
        echo "はんこモデル リポジトリを更新します"
        git pull
    else
        cd /home/$USERNAME/.gazebo/models
        echo "はんこモデル リポジトリをクローンします"
        git clone https://github.com/MasatoKubotera/colorseal_model.git
    fi
fi