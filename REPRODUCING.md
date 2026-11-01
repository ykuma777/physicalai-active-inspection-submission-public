# 保存結果の確認と、動作再現の範囲

## 1. 公開コピーを取得する

~~~sh
git clone https://github.com/ykuma777/physicalai-active-inspection-submission-public.git
cd physicalai-active-inspection-submission-public
python -m pip install -r requirements-offline.txt
python verify_saved_records.py
python verify_package.py
~~~

動作を伴わない保存データの確認です。NumPyは1.24以上3未満を使用します。実際に確認したバージョンはPUBLICATION_QA.jsonに記録します。取得・依存関係の導入後、2本の検証プログラム自体は外部APIを使いません。

verify_saved_records.pyは保存入力6操作に同梱した模倣モデルを適用します。新たなロボット試行や新たな実行時モデル呼出しではありません。モデルを再学習しません。verify_package.pyはhash、学習・調整・評価分離、分類器の画像と正常例の対応、必要ファイルを検査します。両者は確認結果JSONを出力します。

## 2. 学習・分類データ

- 模倣学習元: active_inspection/range_extension/results/campaign_v3_20261030/exports/collection_v1/teacher_actions.jsonl
- 模倣モデルmetadata: active_inspection/range_extension/artifacts/horizontal_bc_v3_20261030.json
- 実際の分割: active_inspection/range_extension/results/campaign_v3_20261030/PLAN.json
- 分類画像: camera_demo/data/images/
- 分類画像manifest: camera_demo/data/dataset_manifest.json
- 分類設定: camera_demo/configs/dataset.yaml と runtime.yaml
- 分類学習実装: camera_demo/ros2_ws/src/camera_inspection/camera_inspection/vision_model.py
- 分類画像の収集実装: camera_demo/scripts/collect_dataset.py

学習画像の拡張は保存画像を加工する処理で、新しい撮影ではありません。同梱した学習コードを公開のために実行・再学習したわけではありません。モデルmetadataの学習元run・時刻・hashは元記録のまま保持しています。

## 3. 動作を再現する場合の環境

記録時の固定コンテナ配置は /home/ubuntu/active_inspection、/home/ubuntu/camera_demo、/home/ubuntu/multigauge です。これらは記録時のコンテナ用ルートで、Windowsホストの個人フォルダーではありません。コードとモデルmetadataのhashを維持するため、必要なランタイムパスを保持しています。

ROS 2 Humble、Ignition Fortress、MoveIt 2、ros2_control、CRANE+のパッケージ、ros_gz_bridge、cv_bridge、Python依存が必要です。生成済みURDF/worldとlaunch.pyは同梱していますが、CRANEのSTLメッシュや外部ROSパッケージ一式は外部依存です。クリーン環境からの構築と実動作は今回検証していません。

単一ゲージの実行接続は range_extension/controller.py と live_v3.py、動画の採用runに使用した接続は active_inspection/deliverables/submission_ready_20261031/work/layout_run_once.py にあります。過去の計画・承認・準備記録を検査し、同じrun IDを再実行しない設計です。過去の対象変更や軌道ログを再送してはいけません。

新しい動作には、新しい配置計画・ID・予算・承認と、現在のdomain/partition、controller、subscriber、新鮮な関節状態、home、命令待機状態、サービス、対象状態の診断が必要です。過去の承認ファイルや個人用の実行準備ファイルは公開していません。安全検査や重複防止を無効化して実行できるようにはしていません。

## 4. 動画・説明資料との対応

- 異常例はire4_repro_layout_20261031_01。模倣2呼出し・2補正・全送信4。既知配置での再現確認で、最終評価へ加算しません。
- 正常例はexamples/normal_classification/。模倣0呼出し・0補正。異なる制御設定の記録で、同一モデルの対照実験ではありません。
- 最終評価は学習・調整に使わなかった左右2配置×固定・規則・模倣の6試行だけです。
- docs/には今回の42ページ説明PDFを1本だけ収録しています。文言修正は実験データやモデルの変更ではありません。

## 5. 残る未検証事項

クリーン環境での動作起動、実機への一般化、広い配置への一般化、q1入力追加の独立した効果、規則方式への優越性は未確認です。分類器の画像別検証報告、実終了反復数、撮影目標の縦座標を選んだ設計理由は保存資料から確定していません。
