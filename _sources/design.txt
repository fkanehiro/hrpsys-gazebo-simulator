========
 設計書
========

hrpsys-baseにgazeboと同期したシミュレーションを実装するために以下の設計を行った。

シミュレーション時間の同期
==========================

gazeboのAPIを用いてシミュレーションループを制御する。これは以下のURLで示されるサンプルと同様の方法である。

https://bitbucket.org/osrf/gazebo/src/0947555ae537f41d02ec3b453cf25454ecc3ec96/examples/stand_alone/custom_main/

なお、上記のAPIのみを使った場合、ROSメッセージを発行するために用いられているプラグインやセンサー処理が動作しないことが報告されているため、センサ処理に関するAPI呼び出し(下記URL参照)の追記も合わせて行なう。

https://bitbucket.org/osrf/gazebo/issue/1145/custom-main-problems-with-time-and-loading

上記ループはソースコード"util/simulator/SimulatorGazebo.cpp"において実装する。


コンポーネントへのシミュレーション時間の伝達
============================================

コンポーネントへのシミュレーション時間の伝達は、"receivers[i]->tick"を用いることで行う。これはhrpsys-baseで用いられている方法と全く同様である。

上記処理はソースコード"util/simulator/SimulatorGazebo.cpp"において実装する。


BodyRTCと同様のインターフェースの実現
=====================================

hrpsys-baseで用いられているBodyRTCを改変することで"GZbodyRTC.cpp"を作成する。

また、hrpsys-baseで用いられているPortHandlerを継承することで、ROSのメッセージ処理を行う"ROSPortHandler.h"と、gazeboと接続する"GZPortHander.h"を作成する。

GZbodyRTCはプロジェクトファイル上の記述に基づいて上記の各ハンドラを読み込む。


注記：OPCODEの重複に関する対処
==============================

hrpsys-baseにリンクされているOpenHRP内で用いられている干渉チェックライブラリOPCODEと、gazeboにリンクされているODE内で用いられているOPCODEのABIが同一でなく、両者をリンクするとプログラムがハングする問題が発生した。そのため以下の対処を行った。

必要なソースコードのコピー
   本来はhrpsys-baseをライブラリとして用いるべきであるが、上記問題を避けるために必要なソース部分をコピーした。具体的には、"BodyState.cpp", "OpenRTMUtil.cpp", "Project.cpp", "ProjectUtil.cpp", "SceneState.cpp"はこの目的で作成されたコピーであり、OPCODEにリンクする必要がある物理シミュレーション関連のコードを消去した以外はオリジナルのコードとの差異はない。

クラスの継承の取りやめ
   本来GZbodyRTCとSimulatorGazeboはそれぞれクラス"BodyRTC"と"Simulator"を継承することが望ましいが、各クラスでOPCODEにリンクする物理シミュレーション関連のコードに依存してしまうため、継承を取りやめた。GZbodyRTCについてはGZbodyRTCBaseという一見不要の親クラスが定義されているが、これは将来上記問題が解決された場合にBodyRTCを継承しやすくするための、物理シミュレーション関連のコードを除いた親クラスである。
