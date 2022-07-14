# フィールド・スコアについて

## 目次

## シミュレーション用ロボットについて

シミュレーションのロボットは次のデバイスで構成されています。

- 2輪差動駆動
    - Twistデータ（/cmd_vel）を受信します。
    - デフォルトではx, yawともに1.0〜-1.0が速度の上限です。
- ラインセンサ
    - 1x100のImageデータ（/camera_linetrace/camera1/image_raw）を送信します。
    - コースアウト判定にも使われるセンサです。
- 前方Lidar
    - -30°〜30°の範囲で50点のLaserScanデータ（/ray/laserscan）を送信します。
    - 距離範囲
        - 最小：0.05m (5cm)
        - 最長：30m
    - スタート・ゴール以外の物体には当たり判定があるため、各物体の距離測定が可能です。
- 自己位置推定
    - `/p3d/odom` を使用します。
    - コマンド＞ `ros2 topic echo /p3d/odom`

- 前方カメラ
    - 640x360のImageデータ（/camera_front/camera_front/image_raw）を送信します。

![Screenshot from 2022-06-22 17-56-55.png](4%20Pub%EF%BC%88Twist%E9%80%81%E4%BF%A1%E3%83%BBC++%EF%BC%89%E2%86%92%E4%BB%BB%E6%84%8F%E3%81%AE%E6%96%B9%E5%90%91%E3%81%AB%E7%A7%BB%E5%8B%95%E3%81%99%E3%82%8B%20c38253adad134812b396ec0cafabd6c0/Screenshot_from_2022-06-22_17-56-55.png)

## フィールドについて

![gazebo-sim.png](./images/score/gazebo-sim.png)

![gazebo.png](./images/score/gazebo.png)

フィールド上に赤い棒が2本立っています。手前側がスタート、GOAL看板の方がゴールとなっており、スタート→ゴールの順で通るとスコアが表示されます。

- スコアは開始時点から減点方式となっています。
- 1秒に1点減点されます。
- ラインから外れると1秒におよそ10点減点されます。
- 残り0点になるかゴールに到達するとcmd_velの操作が効かなくなります。
- ゴールすると、スコアに応じて評価が付きます。（スコア/最大スコアx100）
    - 80% : S
    - 70% : A+
    - 60% : A
    - 50% : B+
    - 40% : B
    - 30% : C+
    - 20% : C
    - 10% : D+
    - 0%以上 : D

## ビルド（Galactic）

```bash
mkdir -p ~/ws_galactic/src
source /opt/ros/galactic/setup.bash
cd ~/ws_galactic/src
git clone git@github.com:HarvestX/h6x-Internship.git
# git clone https://github.com/HarvestX/h6x-Internship.git

# ビルド
cd ../
bash src/h6x-Internship/setup.bash
vcs import src/ < src/h6x-Internship/rvizplugin.repos
colcon build --symlink-install
```

## ワールドの実行

```bash
source ~/ws_galactic/install/setup.bash
ros2 launch h6x_internship_gazebo world.launch.py
```

## 追加予定（未定）

- 時間追加
- 速度上限追加


<br>

[目次へ](./README.md)