# AGENTS.md

## プロジェクト概要

このリポジトリは、CARLA 0.9.16 上で LiDAR ベースの占有グリッドマップ（OGM）を生成し、Ego 車両と RSU の路車間協調認識を評価する研究・シミュレーションプロジェクトです。実験条件、評価コード、オフライン単体テスト、CARLA 実走スクリプトを管理します。

開発単位は **1 Issue = 1 branch = 1 Codex task** とします。`main` は動作確認済みの状態を保ち、直接変更せず Pull Request 経由で統合してください。

## 技術構成と実行環境

- 言語: Python 3.12、PowerShell、Windows batch
- シミュレータ: CARLA 0.9.16（通常は Windows 上の `127.0.0.1:2000`）
- 主なライブラリ: NumPy、Pillow、CARLA Python API
- テスト: pytest
- 設定形式: JSON
- 実験データ既定保存先: `CARLA_DATA_ROOT`。未設定時は `D:\CARLA_DATA`

CARLA サーバ、GPU、GUI、実センサー入力を必要とする統合確認と、CARLA を必要としないオフラインテストを区別してください。CI は後者のみを実行します。

## 主なディレクトリ

- `src/ogm_project/`: OGM、通信、座標変換、診断、実験ランナーの実装
- `scripts/`: 実験、監査、解析のコマンド入口
- `tests/`: CARLA サーバ不要のオフライン単体・回帰テスト
- `configs/`: シナリオと実験条件。履歴上の baseline を含む
- `docs/`: 設計、監査、実験結果の記録
- `legacy/`: 参照用の旧実装。明示的な Issue なしに変更しない
- `outputs/`, `out_grids/`: 生成物置き場。Git 管理しない

## セットアップ

プロジェクトルートで以下を実行します。

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python -m pip install --upgrade pip
.\.venv\Scripts\python -m pip install -r requirements-dev.txt
.\.venv\Scripts\python -m pip install ..\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl
```

CARLA wheel のファイル名は導入環境に合わせて確認してください。CI では CARLA wheel を導入しません。

## ビルド、テスト、lint / format

配布パッケージを生成する build 定義はありません。現在の最低限の build 相当確認は Python の構文コンパイルです。

```powershell
python -m compileall -q src scripts tests
python -m pytest -q
```

JSON 設定は次でも検証できます。

```powershell
Get-ChildItem configs -Filter *.json | ForEach-Object {
    Get-Content -Raw -Encoding UTF8 $_.FullName | ConvertFrom-Json | Out-Null
}
```

専用の lint / formatter はまだ導入されていません。Issue の範囲外で全体を一括整形しないでください。導入する場合は別 Issue とし、既存の数値結果に影響しないことを確認してください。

CARLA 実走の代表コマンドは次のとおりです。

```powershell
python scripts\run_coop_comm.py --scenario-file configs\scenarios.json --scenario scenario_A
python scripts\run_ego_ogm.py --scenario-file configs\scenarios.json --scenario scenario_A
```

## 変更前に確認すること

1. 対象 Issue、完了条件、対象外を読み、対応する作業 branch にいることを確認する。
2. `git status` と `git diff` を確認し、既存の未コミット変更を上書きしない。
3. 関連する `README.md`、`docs/`、`configs/`、テストを先に読む。
4. `legacy/` は参照用であること、`configs/priority_crossing_v1.json` は回帰再現用の固定 baseline であることを確認する。
5. CARLA が必要な変更か、オフラインで検証できる変更かを切り分ける。

## 変更時のルール

- Issue の対象外を勝手に変更しない。不要なリファクタリング、改名、一括整形を避ける。
- 既存アーキテクチャ、実行入口、出力形式、数値的な挙動を可能な限り維持する。
- 履歴再現用の `legacy/` と既存 baseline は原則変更せず、新しい実験条件は `configs/` に別名・別バージョンで追加する。
- 実験パラメータを新たにソースへハードコードせず、再利用する条件は JSON 設定へ分離する。
- 新しい依存ライブラリは必要性、代替案、CI/実行環境への影響を Issue と PR に明記する。
- 秘密情報、認証情報、個人情報、ローカル固有の資格情報をコミットしない。
- 動画、点群、モデル、巨大な配列、生成画像、実験結果一式、大容量ログを不用意に Git 管理しない。必要なら外部ストレージまたは Git LFS を別 Issue で設計する。
- 実験結果には可能な限り、設定ファイル、シナリオ名、乱数 seed、評価条件、出力先、commit hash、必要に応じて Git tag を記録する。

## 変更後に確認すること

1. `python -m compileall -q src scripts tests`
2. `python -m pytest -q`
3. 変更した JSON の parse と、関連する CLI の `--help` / `--list-scenarios`
4. CARLA 連携を変更した場合は、ローカルで対象シナリオを実走し、終了時に actor/sensor が破棄されることと出力先を確認する。
5. `git diff --check`、`git diff`、`git status` で余計な生成物や対象外変更がないことを確認する。
6. 実行できなかった確認は理由とともに PR に明記する。

## branch 命名

- `feature/<name>`: 新機能
- `fix/<name>`: バグ修正
- `refactor/<name>`: 挙動を変えない構造改善
- `docs/<name>`: 文書のみ
- `experiment/<name>`: 実験条件、シナリオ、評価の追加
