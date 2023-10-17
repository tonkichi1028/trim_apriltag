# -*- coding: utf-8 -*-
import csv
import os
from time import sleep
from jtop import jtop

# データを保存するフォルダとファイル名を設定
output_folder = '/home/wanglab/catkin_wsTrim/src/trim_apriltag/data/jtop'
output_filename = 'jtop_2trim.csv'
output_path = os.path.join(output_folder, output_filename)

# 取得するフィールド名を設定
fields = ['CPU1', 'CPU2', 'CPU3', 'CPU4', 'CPU5', 'CPU6', 'GPU', 'RAM', 'SWAP', 'Power TOT', 'Power GPU', 'Temp CPU', 'Temp GPU', 'Temp AUX', 'Temp AO', 'Fan tegra_pwmfan0']

# jtopを開く
with jtop() as jetson:
    # データを書き込むCSVファイルを開く
    with open(output_path, mode='w') as file:
        writer = csv.DictWriter(file, fieldnames=fields)

        # ヘッダー行を書き込む
        writer.writeheader()

        # データを継続的に取得
        while True:
            # システム情報を取得
            stats = jetson.stats

            # 取得した情報から必要なフィールドのみを選択
            row = {field: stats[field] for field in fields}

            # 選択した情報をCSVファイルに書き込む
            writer.writerow(row)

            # 頻繁にデータを取得しすぎないように少し待つ（ここでは1秒待つ）
            sleep(1)

