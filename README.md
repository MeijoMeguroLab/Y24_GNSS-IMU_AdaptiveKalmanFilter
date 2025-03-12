# Y24_GNSS-IMU_AdaptiveKalmanFilter
2024年度作成物、拡張カルマンフィルタをもとに作成したアダプティブカルマンフィルタ

# GNSS/IMU適応型カルマンフィルタ

## 概要

本コードは修士論文研究「RTK-Float解の有効活用に向けた適応型カルマンフィルタ（＝アダプティブカルマンフィルタ）による自己位置推定精度の向上」のために作成したシステムです。GNSS/IMU統合における観測誤差共分散の適応的調整による位置推定精度向上を実現します。特に都市部環境でRTK-FIX解が得られない場合のFloat解活用に焦点を当てています。

**注**: カーブ補正機能（CurveErrorCompensation.m）は実験的な追加機能であり、論文本体の主要部分ではありません。

## 作成者

2024年度修士2年目黒研究室在籍：藤野智史

学籍番号：233432023

## システム要件

- MATLAB R2019b以降
- 推奨ツールボックス：Statistics and Machine Learning Toolbox
- 目黒研究室のMATLAB基幹ソースコードが必要

## ファイル構成

以下のディレクトリ構成でファイルを配置してください：
```
├── a_GIAKF_main.m          # メインスクリプト
├── 0.Data                  # データディレクトリ　サイズが大きいのでNASに保存
│                            \\172.17.10.186\share\00_2024年度研究成果\233432023_藤野\作成物\0.Data
│   ├── observation         # GNSSデータ
│   │   └── nagoya1_float.pos
│   └── 20230616            # リファレンス・IMUデータ
│       ├── POSLV
│       │   └── POSLV_run2.mat
│       └── Eagleye
│           └── run2
│               └── eagleye_log.csv
├── A.Preprocessing         # 前処理モジュール
│   └── A_GIAKF_Preprocessing.m
├── B.ParameterSettings     # パラメータ設定
│   └── B_GIAKF_ParameterSettings.m
├── C.EKF                   # EKF実装
│   └── C_GIAKF_adaptive.m
├── D.Evaluation            # 描画用
├── E.dit                   # 編集用
├── Figure                  # Figure管理フォルダ
├── CurveErrorCompensation.m  # 補足: カーブ補正クラス
└── ImprovedAdaptiveKF.m      # 補足: 拡張適応カルマンフィルタ

```

## 実行方法

1. `a_GIAKF_main.m`をMATLABで開いてください(GIAKF(GNSS/IMU AdaptiveKalmanFilter)以外のファイルは整理されていません)
2. 実行ボタンをクリックしてください
3. パス変更を求めるダイアログが表示された場合、指示に従ってください
4. 結果グラフと分析データが表示されます

## パラメータ設定

パラメータの調整は`B_GIAKF_ParameterSettings.m`で行ってください。主要なパラメータは以下の通りです：

### 基本パラメータ
```matlab
% プロセスノイズパラメータ
param_pros_noise_x     = 0.3;  % x座標のプロセスノイズ
param_pros_noise_y     = 0.3;  % y座標のプロセスノイズ
param_pros_noise_yaw   = 0.0009; % 方位角のプロセスノイズ

% 観測ノイズパラメータ (GNSS Fix時)
param_obs_noise_gnss_x = 0.01;  % x座標の観測ノイズ
param_obs_noise_gnss_y = 0.01;  % y座標の観測ノイズ
param_obs_noise_gnss_yaw = 100/180*pi;  % 方位角の観測ノイズ
```

### 車両パラメータ（カーブ補正使用時）
```matlab
% シエンタ用車両パラメータ
param_vehicle = struct(...
    'wheelBase', 2.75, ...      % ホイールベース [m]
    'cog_height', 0.55, ...     % 重心高さ [m]
    'track_width', 1.49);       % トレッド幅 [m]

% 状態検出パラメータ
param_state_detect = struct(...
    'curve_threshold', 0.1, ... % カーブ判定用ヨーレート閾値 [rad/s]
    'stop_threshold', 0.5);     % 停止判定用速度閾値 [m/s]
```

## 主要変数の説明

### 入力データ
- `data_pos_Obs_enu`: GNSS測位位置（ENU座標系）
- `state_velocity`: 速度データ
- `data_yawrate`: ヨーレート測定値
- `data_pos_Ref_enu`: リファレンス位置（真値）

### 出力結果
- `KF_enu`: 推定位置（ENU座標系）
- `analize_error_EKF3d`: 3D位置誤差
- `status_pseudo_observation_error`: 疑似観測誤差（論文3.1.3節の式(40)に対応）

## 論文との対応

本コードは修士論文の実装部分に対応しており、以下の関連性があります：

1. `status_pseudo_observation_error`: 論文の式(40)で定義される疑似誤差
2. `status_dispersion_param_obs_noise_gnss_x/y`: 論文の式(43)の観測誤差分散
3. `windowSize`: 論文3.2.2で説明される移動窓サイズ（50点≒10秒）
4. `analize_POE`: 論文の式(48)で変換された疑似誤差

## 使用センサ

研究で使用したデータは名古屋大学周辺の都市部環境で取得したものです：
- GNSS受信機: u-blox F9P (5Hz)
- IMUセンサ: Aster社製 (5Hzにダウンサンプリング)
- 真値計測: Applanix社 POS LV（高精度GNSS/INSシステム）

## よくあるエラーと解決方法

### パス関連のエラー
```
Undefined function or variable 'xxx'.
```
**解決策**: MATLABの「Add to Path」機能で必要なフォルダをパスに追加してください。

### データファイル関連のエラー
```
Unable to read file 'xxx'.
```
**解決策**: `a_GIAKF_main.m`内のデータパス設定を確認し、実際のファイル構成と一致するよう修正してください。

### インデックス関連のエラー
```
Index exceeds matrix dimensions.
```
**解決策**: 配列サイズとインデックス参照を確認してください。データの同期や時間軸の不一致が原因の可能性があります。

## カーブ補正機能について

カーブ補正機能（`CurveErrorCompensation.m`）は実験的な追加機能です。以下の特徴があります：

1. 車両のスリップ角を考慮したカーブ走行モデル
2. 路面バンク角の推定と補正
3. ロール動特性の補正

この機能は論文の主題ではありませんが、カーブ区間での精度向上に興味がある場合に参考にしてください。

## 改良の余地

本システムには以下の改良余地があります：

1. カーブ区間でのIMU積分誤差の抑制
2. 適応的な重み係数調整メカニズムの改良
3. 停止状態検出の信頼性向上
4. 計算効率の最適化

また、窓サイズと時間、位置誤差（or スコア）のplot3の三次元プロットを行うべきです。（都合上できなかった）

## おわりに

本コードは研究目的で作成されたものであり、論文で提案した手法の検証を主目的としています。RTK-Float解の有効活用と観測誤差共分散の適応的調整による位置推定精度向上について、実験的な検証を行うための基盤となるものです。

---
※2024年度修士論文研究用に作成
