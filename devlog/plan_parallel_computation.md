# Plan: Parallel Computation for C++ Engine

## Approach
- OpenMP による並列化（標準的、最小限のコード変更）
- 主要ループ（vehicle update, link transfer, route choice）を並列化

## Benchmark
- example_04en_automatic_network_generation.py を使用
- 計測: (1) 現状, (2) 並列実装後の並列速度, (3) 並列実装後のシングルスレッド速度

## Assignment
- Alice (%2): ベースラインベンチマーク取得
- Bob (%1): traffi.cppの主要ループ調査・並列化ポイント特定
- Carol (%3): CMakeLists.txt/ビルド設定のOpenMP対応調査

## Status: IN PROGRESS
