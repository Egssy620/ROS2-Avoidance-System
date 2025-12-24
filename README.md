# ROS2-Avoidance-System

ROS2 Avoidance System は、ROS2 上で動作する自律移動ロボット向けの障害物回避フレームワークです。本プロジェクトは複数のノードで構成され、LiDARセンサを用いた距離情報処理、経路追従、姿勢調整、回避戦略を統合します。

本リポジトリは、実機およびシミュレーション環境での自律走行・障害物回避タスクに対応する設計になっています。

---

## 🔎 特徴

- LiDARによる障害物検知・回避
- 目標姿勢への誘導とモードベース制御
- 複数独立ノード構成による拡張性
- ROS2 (Foxy / Humble / Iron など) に対応

---

## 📁 ディレクトリ構成

| ディレクトリ／ノード | 機能概要 |
|----------------------|------------------------------|
| `lidar_pkg`          | LiDARトピック処理 & 障害物検知 |
| `drive_node`         | 前進・回避指令生成ノード |
| `heading_test_node`  | 見込み方向の姿勢評価 |
| `cmd_vel_mux`        | 複数ソースの速度指令合成 |
| `manage_node`        | 状態遷移管理（回避・走行統合制御） |
| `task_node`          | 上位指令・ミッション制御 |
| `aruco_pnp_node`     | ArUcoマーカーによる位置推定（オプション） |
| `ros2_launch_ws`     | 起動用Launchファイル集 |

---

## 🛠️ 必要条件

以下の環境を前提とします：

- Ubuntu 20.04 / 22.04
- ROS2 Foxy / Humble / Iron
- colcon ビルドツール
- LiDARドライバ（Velodyne, RPLIDAR など）

---

## 📦 インストール手順

### 1. ワークスペースを作成

```bash
mkdir -p ~/ros2_avoidance_ws/src
cd ~/ros2_avoidance_ws/src
