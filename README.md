# MoveItの実行

このドキュメントでは、Toyota HSR (Human Support Robot) を用いてMoveItを実行するための手順を説明します。

## HSRの起動

以下のコマンドを実行して、HSRをRVizとシミュレータで起動します。

```bash
ros2 launch hsrb_rviz_simulator hsrb_rviz_simulator.launch.py
```

## MoveItのアクションサーバー起動

次に、MoveItのアクションサーバーを起動します。以下のコマンドを実行してください。

```bash
ros2 launch hsrb_moveit_config moveit_service_server.launch.py
```

## テスト

以下のコマンドを使用して、MoveItの機能をテストできます。

### 手先を特定座標へ移動させる (サービス呼び出し)

以下のコマンドを実行すると、HSRの手先が指定された座標へ移動します。

```bash
ros2 service call /move_hand_to_target_coord hsrb_moveit_interfaces/srv/MoveHandToTargetCoord "pose:
  position:
    x: 1.0
    y: 1.0
    z: 0.7
  orientation:
    x: 0.707
    y: 0.0
    z: 0.707
    w: 0.0"
```

### MoveIt IKデモの起動

以下のコマンドを実行すると、MoveItの逆運動学 (IK) のデモが起動します。

```bash
ros2 launch hsrb_moveit_config hsrb_example.launch.py example_name:=moveit_ik_demo
```

### tfフレームに基づいて手先を移動させる (サービス呼び出し)

以下のコマンドを実行すると、publishされたtfフレームに基づいてHSRの手先が移動します。

まず、移動目標となるtfフレームをpublishします。

```bash
ros2 run tf2_ros static_transform_publisher 2.0 1.0 0.7 0.707 0.0 0.707 0.0 odom goal
```

次に、`/move_hand_to_target_tf` サービスを呼び出し、目標のtfフレーム名を指定します。

```bash
ros2 service call /move_hand_to_target_tf hsrb_moveit_interfaces/srv/MoveHandToTargetTf "object_name: 'goal'"
```

## サービスインターフェース定義

以下は、テストで使用したサービスのインターフェース定義です。

### move_hand_to_target_coord サービス

```
geometry_msgs/Pose pose
  Point position
    float64 x
    float64 y
    float64 z
  Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
---
bool success
```

### move_hand_to_target_tf サービス

```
string object_name
---
bool success
```