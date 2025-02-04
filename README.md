# 課題1: ROS制御

ROS Version: Humble
実装言語: Python
動作確認環境: Ubuntu 24.04上でのDocker container


## ビルド方法

### Dockerをインストール

[公式ドキュメント](https://docs.docker.com/engine/install/ubuntu/)に従い、Dockerをインストールします。


### リポジトリをクローン

ターミナルで下記コマンドを実行します。

```
git clone https://github.com/kaz399/ros2_experimental.git
```

### Dockerイメージの作成

ターミナルで下記コマンドを実行します。

```
cd ros2_experimental
./utils/build-container.sh
```

### コンテナの起動

ターミナルで下記コマンドを実行し、Dockerコンテナの中に入ります。

```
./utils/run-container.sh
```

### プログラムの実行（動作確認）

Dockerコンテナ内で下記コマンドを実行します。

```
./rostest.sh
```

Gazeboが起動し、しばらく待つとロボットが動き出します。



